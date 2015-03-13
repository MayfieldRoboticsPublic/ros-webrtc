#include "host.h"

#include <talk/app/webrtc/peerconnectioninterface.h>
#include <talk/app/webrtc/videosourceinterface.h>
#include <talk/media/devices/devicemanager.h>
#include <talk/media/webrtc/webrtcvideoencoderfactory.h>
#include <talk/media/webrtc/webrtcvideodecoderfactory.h>

#include "device_manager.h"
#include "host.h"
#include "util.h"


// VideoSource

VideoSource::VideoSource() :
    type(NoneType),
    publish(false) {
}

VideoSource::VideoSource(
    Type type,
    const std::string& name,
    const std::string& label,
    const MediaConstraints& constraints,
    bool publish
    ) :
    type(type),
    name(name),
    label(label),
    constraints(constraints),
    publish(publish) {
}

// AudioSource

AudioSource::AudioSource() : publish(false) {
}

AudioSource::AudioSource(
    const std::string& label_,
    const MediaConstraints& constraints_,
    bool publish_
    ) :
    label(label_),
    constraints(constraints_),
    publish (publish_) {
}

// HostFactory

Host HostFactory::operator()(ros::NodeHandle &nh) {
    return Host(
        nh,
        video_srcs,
        audio_src,
        session_constraints,
        ice_servers
    );
}

// Host

Host::Host(
    ros::NodeHandle& nh,
    const std::vector<VideoSource>& video_srcs,
    const AudioSource& audio_src,
    const MediaConstraints& session_constraints,
    const std::vector<webrtc::PeerConnectionInterface::IceServer>& ice_servers
    ) :
    _nh(nh),
    _video_srcs(video_srcs),
    _audio_src(audio_src),
    _session_constraints(session_constraints),
    _ice_servers(ice_servers),
    _srv(*this) {
}

Host::Host(const Host& other) :
    _nh(other._nh),
    _video_srcs(other._video_srcs),
    _audio_src(other._audio_src),
    _session_constraints(other._session_constraints),
    _ice_servers(other._ice_servers),
    _srv(*this) {
}

Host::~Host() {
    close();
}

bool Host::open() {
    if (!_create_pc_factory()) {
        close();
        return false;
    }
    if (!_open_local_stream()) {
        close();
        return false;
    }
    _srv.advertise();
    return true;
}

bool Host::is_open() const {
    return _pc_factory.get() != NULL;
}

void Host::close() {
    _srv.shutdown();
    _close_local_stream();
    _pc_factory.release();
    _worker_thd.reset();
    _signaling_thd.reset();
}

SessionPtr Host::begin_session(
    const std::string& id,
    const std::string& peer_id,
    const MediaConstraints& sdp_constraints,
    const std::vector<ros_webrtc::DataChannel>& data_channels,
    const std::map<std::string, std::string>& service_names
    ) {
    ROS_INFO("creating session id='%s', peer='%s'", id.c_str(), peer_id.c_str());
    SessionKey key = {id, peer_id};
    if (_sessions.find(key) != _sessions.end()) {
        ROS_ERROR("session w/ id='%s', peer='%s' already exists", id.c_str(), peer_id.c_str());
        return SessionPtr();
    }
    SessionPtr s(new Session(
        id,
        peer_id,
        _local_stream,
        sdp_constraints,
        data_channels,
        service_names
    ));
    Session::ObserverPtr pc_observer(new SessionObserver(*this, s));
    if (!s->begin(
        _pc_factory,
        &_session_constraints,
        _ice_servers,
        pc_observer
        ))
        return SessionPtr();
    _sessions[key] = s;
    return s;
}

bool Host::end_session(const std::string& id, const std::string& peer_id) {
    SessionKey key = {id, peer_id};
    auto i = _sessions.find(key);
    if (i == _sessions.end()) {
        ROS_INFO("no session w/ id='%s' peer='%s'", id.c_str(), peer_id.c_str());
        return false;
    }
    (*i).second->end();
    _sessions.erase(i);
    return true;
}

Host::FlushStats Host::flush() {
    FlushStats flush;
    for (auto i = _sessions.begin(); i != _sessions.end(); i++) {
        auto session_flush = (*i).second->flush();
        flush += session_flush;
    }
    return flush;
}

bool Host::_create_pc_factory() {
    _worker_thd.reset(new rtc::Thread());
    _worker_thd->SetName("worker_thread", NULL);
    if (!_worker_thd->Start()) {
        ROS_DEBUG_STREAM("worker thread failed to start");
        close();
        return false;
    }

    _signaling_thd.reset(new rtc::Thread());
    _signaling_thd->SetName("signaling_thread", NULL);
    if (!_signaling_thd->Start()) {
        ROS_DEBUG_STREAM("signaling thread failed to start");
        close();
        return false;
    }

    rtc::scoped_ptr<cricket::WebRtcVideoEncoderFactory> encoder_factory;
    rtc::scoped_ptr<cricket::WebRtcVideoDecoderFactory> decoder_factory;
    _pc_factory =  webrtc::CreatePeerConnectionFactory(
        _worker_thd.get(),
        _signaling_thd.get(),
        NULL,
        encoder_factory.release(),
        decoder_factory.release()
    );
    if (!_pc_factory.get()) {
        ROS_DEBUG_STREAM("failed peer-connection factory create");
        close();
        return false;
    }

    return true;
}

bool Host::_open_local_stream() {
    // stream
    std::stringstream ss;
    ss << "s" << 1;
    std::string stream_label = ss.str();
    ss.str("");
    ss.clear();
    _local_stream = _pc_factory->CreateLocalMediaStream(stream_label);

    ROS_DEBUG_STREAM("creating system device manager");
    rtc::scoped_ptr<cricket::DeviceManagerInterface> sys_dev_mgr(
        cricket::DeviceManagerFactory::Create()
    );
    if (!sys_dev_mgr->Init()) {
        ROS_ERROR_STREAM("cannot create system device manager");
        return false;
    }

    ROS_DEBUG_STREAM("creating ros device manager");
    ROSVideoCaptureTopicInfoPtr video_capture_topics(new ROSVideoCaptureTopicInfo());
    for (size_t i = 0; i != _video_srcs.size(); i++) {
        VideoSource& video_src = _video_srcs[i];
        if (video_src.type != VideoSource::ROSType)
            continue;
        video_capture_topics->add(video_src.name);
    }
    rtc::scoped_ptr<cricket::DeviceManagerInterface> ros_dev_mgr(
        new ROSDeviceManager(video_capture_topics)
    );
    if (!ros_dev_mgr->Init()) {
        ROS_ERROR_STREAM("cannot create ros device manager");
        return false;
    }

    // audio track
    std::string audio_label = _audio_src.label;
    if (audio_label.empty()) {
        ss << "a" << 1;
        audio_label = ss.str();
        ss.str("");
        ss.clear();
    }
    rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track(
        _pc_factory->CreateAudioTrack(
           audio_label,
           _pc_factory->CreateAudioSource(&_audio_src.constraints)
        )
    );
    if(audio_track.get() == NULL) {
        ROS_ERROR("cannot create track '%s'", audio_label.c_str());
        return false;
    }
    if (_audio_src.publish) {
        _audio_sink.reset(new AudioSink(
            _nh,
            topic_for({"local", "audio_" + audio_track->id()}),
            audio_track
        ));
    }
    _local_stream->AddTrack(audio_track);

    // video tracks
    for (size_t i = 0; i != _video_srcs.size(); i++) {
        const VideoSource& video_src = _video_srcs[i];

        // device manager
        cricket::DeviceManagerInterface *dev_mgr = NULL;
        switch (video_src.type) {
            case VideoSource::SystemType:
                dev_mgr = sys_dev_mgr.get();
                break;
            case VideoSource::ROSType:
                dev_mgr = ros_dev_mgr.get();
                break;
            default:
                ROS_ERROR("video source '%s' type '%d' not supported",video_src.name.c_str(), video_src.type);
                return false;
        }

        // capturer
        cricket::Device device;
        if (!dev_mgr->GetVideoCaptureDevice(video_src.name, &device)) {
            ROS_ERROR("cannot get video capture device for '%s'", video_src.name.c_str());
            return false;
        }
        std::auto_ptr<cricket::VideoCapturer> video_capturer(dev_mgr->CreateVideoCapturer(device));
        if (video_capturer.get() == NULL) {
            ROS_ERROR("failed to create video capture device for '%s'", video_src.name.c_str());
            return false;
        }

        // track
        std::string video_label = video_src.label;
        if (video_label.empty()) {
            ss << "v" << i + 1;
            video_label = ss.str();
            ss.str("");
            ss.clear();
        }
        rtc::scoped_refptr<webrtc::VideoTrackInterface> video_track(
            _pc_factory->CreateVideoTrack(
                video_label, _pc_factory->CreateVideoSource(video_capturer.get(), &video_src.constraints)
            )
        );
        if(video_track.get() == NULL) {
            ROS_ERROR(
                "cannot create track '%s' for video capture device '%s'",
                video_label.c_str(), video_src.name.c_str()
            );
            return false;
        }
        video_capturer.release();
        if (video_src.publish) {
            VideoRendererPtr video_renderer(new VideoRenderer(
                _nh, topic_for({"local", "video_" + video_track->id()}), video_track
            ));
            _video_renderers.push_back(video_renderer);
        }
        _local_stream->AddTrack(video_track);
    }

    return true;
}

void Host::_close_local_stream() {
    _audio_sink.reset();
    while (!_video_renderers.empty()) {
        _video_renderers.pop_front();
    }
    _local_stream = NULL;
}

SessionPtr Host::_find_session(const SessionKey& key) {
    auto i = _sessions.find(key);
    if (i == _sessions.end())
        return SessionPtr();
    return (*i).second;
}

SessionConstPtr Host::_find_session(const SessionKey& key) const {
    auto i = _sessions.find(key);
    if (i == _sessions.end())
        return SessionConstPtr();
    return (*i).second;
}

// Host::Flush

Host::FlushStats& Host::FlushStats::operator += (const Session::FlushStats & rhs) {
    reaped_data_messages += rhs.reaped_data_messages;
    return *this;
}

// Host::Service

Host::Service::Service(Host& instance) : _instance(instance) {
}

void Host::Service::advertise() {
    _srvs.push_back(_instance._nh.advertiseService(service_for("begin_session"), &Host::Service::begin_session, this));
    _srvs.push_back(_instance._nh.advertiseService(service_for("end_session"), &Host::Service::end_session, this));
    _srvs.push_back(_instance._nh.advertiseService(service_for("connect_session"), &Host::Service::connect_session, this));
    _srvs.push_back(_instance._nh.advertiseService(service_for("add_session_ice_candidate"), &Host::Service::add_session_ice_candidate, this));
    _srvs.push_back(_instance._nh.advertiseService(service_for("set_session_description"), &Host::Service::set_session_description, this));
    _srvs.push_back(_instance._nh.advertiseService(service_for("get_session"), &Host::Service::get_session, this));
    _srvs.push_back(_instance._nh.advertiseService(service_for("get_host"), &Host::Service::get_host, this));
}

void Host::Service::shutdown() {
    _srvs.clear();
}

bool Host::Service::begin_session(ros::ServiceEvent<ros_webrtc::BeginSession::Request, ros_webrtc::BeginSession::Response>& event) {
    const auto &req = event.getRequest();
    MediaConstraints sdp_constraints;
    for(size_t i = 0; i != req.sdp_constraints.mandatory.size(); i++) {
        sdp_constraints.mandatory().push_back(MediaConstraints::Constraint(
            req.sdp_constraints.mandatory[i].key,
            req.sdp_constraints.mandatory[i].value
        ));
    }
    for(size_t i = 0; i != req.sdp_constraints.optional.size(); i++) {
        sdp_constraints.optional().push_back(MediaConstraints::Constraint(
            req.sdp_constraints.optional[i].key,
            req.sdp_constraints.optional[i].value
        ));
    }
    std::map<std::string, std::string> service_names;
    service_names["end_session"] = req.end_session;
    service_names["add_session_ice_candidate"] = req.add_session_ice_candidate;
    service_names["set_session_description"] = req.set_session_description;
    SessionPtr session(_instance.begin_session(
        req.session_id, req.peer_id, sdp_constraints, req.data_channels, service_names
    ));
    return session != NULL;
}

bool Host::Service::end_session(ros::ServiceEvent<ros_webrtc::EndSession::Request, ros_webrtc::EndSession::Response>& event) {
    const auto& req = event.getRequest();
    return _instance.end_session(req.session_id, req.peer_id);
}

bool Host::Service::connect_session(ros::ServiceEvent<ros_webrtc::ConnectSession::Request, ros_webrtc::ConnectSession::Response>& event) {
    const auto& req = event.getRequest();
    SessionKey key = {req.session_id, req.peer_id};
    SessionPtr session = _instance._find_session(key);
    if (session == NULL)
        return false;
    if (!session->create_offer()) {
        return false;
    }
    return true;
}

bool Host::Service::add_session_ice_candidate(ros::ServiceEvent<ros_webrtc::AddSessionIceCandidate::Request, ros_webrtc::AddSessionIceCandidate::Response>& event) {
    const auto& req = event.getRequest();
    SessionKey key = {req.session_id, req.peer_id};
    SessionPtr session = _instance._find_session(key);
    if (session == NULL)
        return false;
    rtc::scoped_ptr<webrtc::IceCandidateInterface> ice_candidate(webrtc::CreateIceCandidate(
        req.sdp_mid, req.sdp_mline_index, req.candidate
    ));
    session->add_remote_ice_candidate(ice_candidate.get());
    return true;
}

bool Host::Service::set_session_description(ros::ServiceEvent<ros_webrtc::SetSessionDescription::Request, ros_webrtc::SetSessionDescription::Response>& event) {
    const auto& req = event.getRequest();
    SessionKey key = {req.session_id, req.peer_id};
    SessionPtr session = _instance._find_session(key);
    if (session == NULL)
        return false;
    auto desc = webrtc::CreateSessionDescription(req.type, req.sdp);
    session->set_remote_session_description(desc);
    if (!session->is_offerer()) {
        session->create_answer();
    }
    return true;
}

bool Host::Service::get_session(ros::ServiceEvent<ros_webrtc::GetSession::Request, ros_webrtc::GetSession::Response>& event) {
    const auto &req = event.getRequest();
    SessionKey key = {req.session_id, req.peer_id};
    SessionPtr session = _instance._find_session(key);
    if (session == NULL)
        return false;
    auto &resp = event.getResponse();
    webrtc::scoped_refptr<webrtc::PeerConnectionInterface> pc(session->peer_connection());
    if (pc.get() != NULL) {
        switch (session->peer_connection()->signaling_state()) {
            case webrtc::PeerConnectionInterface::SignalingState::kStable:
                resp.signaling_state = "stable";
                break;
            case webrtc::PeerConnectionInterface::SignalingState::kHaveLocalOffer:
                resp.signaling_state = "have_local_offer";
                break;
            case webrtc::PeerConnectionInterface::SignalingState::kHaveLocalPrAnswer:
                resp.signaling_state = "have_local_pr_answer";
                break;
            case webrtc::PeerConnectionInterface::SignalingState::kHaveRemoteOffer:
                resp.signaling_state = "have_remote_offer";
                break;
            case webrtc::PeerConnectionInterface::SignalingState::kHaveRemotePrAnswer:
                resp.signaling_state = "have_remote_pr_answer";
                break;
            case webrtc::PeerConnectionInterface::SignalingState::kClosed:
                resp.signaling_state = "closed";
                break;
        };
    }
    return true;
}

bool Host::Service::get_host(ros::ServiceEvent<ros_webrtc::GetHost::Request, ros_webrtc::GetHost::Response>& event) {
    const auto &req = event.getRequest();
    auto &resp = event.getResponse();
    for (auto i = _instance._session_constraints.mandatory().begin();
         i != _instance._session_constraints.mandatory().end();
         i++) {
        ros_webrtc::Constraint constraint;
        constraint.key = (*i).key;
        constraint.value = (*i).value;
        resp.sdp_constraints.mandatory.push_back(constraint);
    }
    for (auto i = _instance._session_constraints.optional().begin();
         i != _instance._session_constraints.optional().end();
         i++) {
        ros_webrtc::Constraint constraint;
        constraint.key = (*i).key;
        constraint.value = (*i).value;
        resp.sdp_constraints.optional.push_back(constraint);
    }
    webrtc::AudioTrackVector audio_tracks = _instance._local_stream->GetAudioTracks();
    for (auto i = audio_tracks.begin(); i != audio_tracks.end(); i++) {
        ros_webrtc::Track track;
        track.kind = (*i)->kind();
        track.id = (*i)->id();
        switch ((*i)->state()) {
            case webrtc::MediaStreamTrackInterface::kInitializing:
                track.state = "initializing";
                break;
            case webrtc::MediaStreamTrackInterface::kLive:
                track.state = "live";
                break;
            case webrtc::MediaStreamTrackInterface::kEnded:
                track.state = "ended";
                break;
            case webrtc::MediaStreamTrackInterface::kFailed:
                track.state = "failed";
                break;
            default:
                track.state = "unknown";
                break;
        }
        track.enabled = (*i)->enabled();
        resp.tracks.push_back(track);
    }
    webrtc::VideoTrackVector video_tracks = _instance._local_stream->GetVideoTracks();
    for (auto i = video_tracks.begin(); i != video_tracks.end(); i++) {
        ros_webrtc::Track track;
        track.kind = (*i)->kind();
        track.id = (*i)->id();
        switch ((*i)->state()) {
            case webrtc::MediaStreamTrackInterface::kInitializing:
                track.state = "initializing";
                break;
            case webrtc::MediaStreamTrackInterface::kLive:
                track.state = "live";
                break;
            case webrtc::MediaStreamTrackInterface::kEnded:
                track.state = "ended";
                break;
            case webrtc::MediaStreamTrackInterface::kFailed:
                track.state = "failed";
                break;
            default:
                track.state = "unknown";
                break;
        }
        track.enabled = (*i)->enabled();
        resp.tracks.push_back(track);
    }
    for (auto i = _instance._sessions.begin(); i != _instance._sessions.end(); i++) {
        ros_webrtc::SessionKey key;
        key.id = (*i).second->id();
        key.peer_id = (*i).second->peer_id();
        resp.sessions.push_back(key);
    }
    return true;
}

// Host::EndSessionCallback

Host::EndSessionCallback::EndSessionCallback(Host& instance, const SessionKey& key) :
    _instance(instance),
    _key(key) {
}

ros::CallbackInterface::CallResult Host::EndSessionCallback::call() {
    _instance.end_session(_key.id, _key.peer_id);
    return Success;
}


// Host::SessionObserver

Host::SessionObserver::SessionObserver(Host& instance, SessionPtr session) :
    _instance(instance),
    _session(session) {
}

Host::SessionObserver::~SessionObserver() {
}

void Host::SessionObserver::on_connection_change(webrtc::PeerConnectionInterface::IceConnectionState state) {
    if (state == webrtc::PeerConnectionInterface::kIceConnectionDisconnected) {
        SessionKey key = {_session->id(), _session->peer_id()};
        ros::CallbackInterfacePtr callback(new Host::EndSessionCallback(_instance, key));
        _instance._nh.getCallbackQueue()->addCallback(callback);
    }
}
