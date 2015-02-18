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

Host HostFactory::operator()() {
    return Host(
        video_srcs,
        audio_src,
        session_constraints,
        ice_servers
    );
}

// Host

Host::Host(
    const std::vector<VideoSource>& video_srcs,
    const AudioSource& audio_src,
    const MediaConstraints& session_constraints,
    const std::vector<webrtc::PeerConnectionInterface::IceServer>& ice_servers
    ) :
    _video_srcs(video_srcs),
    _audio_src(audio_src),
    _session_constraints(session_constraints),
    _ice_servers(ice_servers) {
}

Host::Host(const Host& other) :
    _video_srcs(other._video_srcs),
    _audio_src(other._audio_src),
    _session_constraints(other._session_constraints),
    _ice_servers(other._ice_servers) {
}

Host::~Host() {
    close();
}

bool Host::open() {
    _dc_rpub = _nh.advertise<ros_webrtc::Data>(topic_for("data_recv"), 1000, false);
    if (!_create_pc_factory()) {
        close();
        return false;
    }
    if (!_open_local_stream()) {
        close();
        return false;
    }
    if (!_open_servers()) {
        close();
        return false;
    }
    return true;
}

bool Host::is_open() const {
    return _pc_factory.get() != NULL;
}

void Host::close() {
    _close_servers();
    _close_local_stream();
    _pc_factory.release();
    _worker_thd.reset();
    _signaling_thd.reset();
    _dc_rpub.shutdown();
}

SessionPtr Host::begin_session(
    const std::string& peer_id,
    const MediaConstraints& sdp_constraints,
    const std::vector<ros_webrtc::DataChannel>& data_channels,
    const std::map<std::string, std::string>& service_names
    ) {
    ROS_INFO_STREAM("creating session for peer " << peer_id);
    SessionPtr s(new Session(
        peer_id,
        _local_stream,
        sdp_constraints,
        _dc_rpub,
        data_channels,
        service_names
    ));
    Session::ObserverPtr pc_observer(
        new SessionObserver(*this, s)
    );
    if (!s->begin(_pc_factory, &_session_constraints, _ice_servers, pc_observer)) {
        return SessionPtr();
    }
    _sessions.push_back(s);
    return s;
}

bool Host::end_session(const std::string& peer_id) {
    for (Sessions::iterator i = _sessions.begin(); i != _sessions.end(); i++) {
        if ((*i)->peer_id() == peer_id) {
            ROS_INFO_STREAM("ending session for peer '" << peer_id << "'");
            (*i)->end();
            _sessions.erase(i);
            return true;
        }
    }
    ROS_INFO_STREAM("not session for for peer '" << peer_id << "' to end");
    return false;
}

Host::Sessions& Host::sessions() {
    return _sessions;
}

const Host::Sessions& Host::sessions() const {
    return _sessions;
}

Host::Flush Host::flush() {
    Flush flush;
    for (Sessions::iterator i = _sessions.begin(); i != _sessions.end(); i++) {
        Session::Flush session_flush = (*i)->flush();
        flush += session_flush;
    }
    return flush;
}

SessionPtr Host::session(const std::string& peer_id) {
    for (Sessions::iterator i = _sessions.begin(); i != _sessions.end(); i++) {
        if ((*i)->peer_id() == peer_id) {
            return (*i);
        }
    }
    return SessionPtr();
}

SessionConstPtr Host::session(const std::string& peer_id) const {
    for (Sessions::const_iterator i = _sessions.begin(); i != _sessions.end(); i++) {
        if ((*i)->peer_id() == peer_id) {
            return (*i);
        }
    }
    return SessionConstPtr();
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
    rtc::scoped_ptr<cricket::DeviceManagerInterface> sys_dev_mgr(cricket::DeviceManagerFactory::Create());
    if (!sys_dev_mgr->Init()) {
        ROS_ERROR_STREAM("cannot create system device manager");
        return false;
    }

    ROS_DEBUG_STREAM("creating ros device manager");
    rtc::scoped_ptr<cricket::DeviceManagerInterface> ros_dev_mgr(new ROSDeviceManager());
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
            _nh, topic_for("local", "audio_" + audio_track->id()), audio_track
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
                _nh, topic_for("local", "video_" + video_track->id()), video_track
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

bool Host::_open_servers() {
    _rsrvs.push_back(_nh.advertiseService(service_for("connect"), &Host::_serve_connect, this));
    _rsrvs.push_back(_nh.advertiseService(service_for("disconnect"), &Host::_serve_disconnect, this));
    _rsrvs.push_back(_nh.advertiseService(service_for("ice_candidate"), &Host::_serve_ice_candidate, this));
    _rsrvs.push_back(_nh.advertiseService(service_for("sdp_offer_answer"), &Host::_serve_sdp_offer_answer, this));
    _rsrvs.push_back(_nh.advertiseService(service_for("sessions"), &Host::_serve_sessions, this));
    _rsubs.push_back(_nh.subscribe(topic_for("data_send"), 1000, &Host::_handle_send, this));
    return true;
}

void Host::_close_servers() {
    _rsrvs.clear();
    _rsubs.clear();
}

bool Host::_serve_connect(ros::ServiceEvent<ros_webrtc::Connect::Request, ros_webrtc::Connect::Response>& event) {
    const ros_webrtc::Connect::Request &req = event.getRequest();
    ROS_INFO_STREAM("serve 'connect' for peer " << req.peer_id);
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
    service_names.insert(
        std::map<std::string, std::string>::value_type("disconnect", req.disconnect_service)
    );
    service_names.insert(
        std::map<std::string, std::string>::value_type("ice_candidate", req.ice_candidate_service)
    );
    service_names.insert(
        std::map<std::string, std::string>::value_type("sdp_offer_answer", req.sdp_offer_answer_service)
    );
    SessionPtr session(begin_session(
        req.peer_id, sdp_constraints, req.data_channels, service_names
    ));
    if (!session->create_offer()) {
        end_session(req.peer_id);
        return false;
    }
    return true;
}

bool Host::_serve_disconnect(ros::ServiceEvent<ros_webrtc::Disconnect::Request, ros_webrtc::Disconnect::Response>& event) {
    const ros_webrtc::Disconnect::Request& req = event.getRequest();
    ROS_INFO_STREAM("serve 'disconnect' for peer " << req.peer_id);
    SessionPtr s(session(req.peer_id));
    if (s == NULL) {
        ROS_INFO_STREAM("no session for peer " << req.peer_id);
        return false;
    }
    end_session(req.peer_id);
    return true;
}

bool Host::_serve_ice_candidate(ros::ServiceEvent<ros_webrtc::IceCandidate::Request, ros_webrtc::IceCandidate::Response>& event) {
    const ros_webrtc::IceCandidate::Request &req = event.getRequest();
    ROS_INFO_STREAM("serve 'ice_candidate' for peer " << req.peer_id);
    SessionPtr s(session(req.peer_id));
    if (s == NULL) {
        ROS_INFO_STREAM("no session for peer " << req.peer_id);
        return false;
    }
    rtc::scoped_ptr<webrtc::IceCandidateInterface> ice_candidate(webrtc::CreateIceCandidate(
        req.sdp_mid, req.sdp_mline_index, req.candidate
    ));
    s->add_remote_ice_candidate(ice_candidate.get());
    return true;
}

bool Host::_serve_sdp_offer_answer(ros::ServiceEvent<ros_webrtc::SdpOfferAnswer::Request, ros_webrtc::SdpOfferAnswer::Response>& event) {
    const ros_webrtc::SdpOfferAnswer::Request &req = event.getRequest();
    ROS_INFO_STREAM("serve 'sdp_offer_answer' for peer " << req.peer_id);
    SessionPtr s(session(req.peer_id));
    if (s == NULL) {
        ROS_INFO_STREAM("no session for peer " << req.peer_id);
        return false;
    }
    webrtc::SessionDescriptionInterface* desc = webrtc::CreateSessionDescription(req.type, req.sdp);
    s->set_remote_session_description(desc);
    if (!s->is_offerer()) {
        s->create_answer();
    }
    return true;
}

bool Host::_serve_sessions(ros::ServiceEvent<ros_webrtc::Sessions::Request, ros_webrtc::Sessions::Response>& event) {
    Host::Sessions ss(sessions());
    ros_webrtc::Sessions::Response& resp = event.getResponse();
    for(Host::Sessions::iterator i = ss.begin(); i != ss.end(); i++)  {
        resp.peer_ids.push_back((*i)->peer_id());
    }
    return true;
}

void Host::_handle_send(const ros_webrtc::DataConstPtr& msg) {
    webrtc::DataBuffer data_buffer(
        rtc::Buffer(&msg->buffer[0], msg->buffer.size()),
        msg->encoding == "binary"
    );
    Host::Sessions ss(sessions());
    for(Host::Sessions::iterator i = ss.begin(); i != ss.end(); i++)  {
        Session::DataChannel* data_channel = (*i)->data_channel(msg->label);
        if (data_channel == NULL) {
            continue;
        }
        data_channel->send(data_buffer);
    }
}

// Host::Flush

Host::Flush& Host::Flush::operator += (const Session::Flush & rhs) {
    reaped_data_messages += rhs.reaped_data_messages;
    return *this;
}

// Host::SessionObserver

Host::SessionObserver::SessionObserver(
    Host& instance_,
    SessionPtr session_
    ) : instance(instance_), session(session_) {
}

Host::SessionObserver::~SessionObserver() {
}

void Host::SessionObserver::on_connection_change(webrtc::PeerConnectionInterface::IceConnectionState state) {
    if (state == webrtc::PeerConnectionInterface::kIceConnectionDisconnected) {
        instance.end_session(session->peer_id());
    }
}
