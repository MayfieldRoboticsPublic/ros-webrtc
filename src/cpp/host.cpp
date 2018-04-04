#include "host.h"

#include <webrtc/api/peerconnectioninterface.h>
#include <webrtc/media/base/videosourceinterface.h>
#include <webrtc/media/engine/webrtcvideoencoderfactory.h>

#include "convert.h"
#include "host.h"
#include "util.h"


// VideoSource

VideoSource::VideoSource() :
    type(NameType),
    publish(false),
    rotation(0) {
}

VideoSource::VideoSource(
    Type type,
    const std::string& name,
    const std::string& label,
    const MediaConstraints& constraints,
    bool publish,
    int rotation) :
    type(type),
    name(name),
    label(label),
    constraints(constraints),
    publish(publish),
    rotation(rotation) {
}

// AudioSource

AudioSource::AudioSource() : publish(false) {
}

AudioSource::AudioSource(
    const std::string& label_,
    const MediaConstraints& constraints_,
    bool publish_) :
    label(label_),
    constraints(constraints_),
    publish (publish_) {
}

// QueueSizes

QueueSizes::QueueSizes(uint32_t size) :
    audio(size),
    video(size),
    data(size),
    event(size) {
}

QueueSizes::QueueSizes(uint32_t video, uint32_t audio, uint32_t data, uint32_t event) :
    audio(audio),
    video(video),
    data(data),
    event(event) {
}

// Host

Host::Host(
    ros::NodeHandle& nh,
    const std::vector<VideoSource>& video_srcs,
    const AudioSource& audio_src,
    const MediaConstraints& pc_constraints,
    double pc_bond_connect_timeout,
    double pc_bond_heartbeat_timeout,
    const std::vector<webrtc::PeerConnectionInterface::IceServer>& default_ice_servers,
    const QueueSizes& queue_sizes) :
    _nh(nh),
    _video_srcs(video_srcs),
    _video_capture_modules(new VideoCaptureModuleRegistry()),
    _audio_src(audio_src),
    _pc_constraints(pc_constraints),
    _pc_bond_connect_timeout(pc_bond_connect_timeout),
    _pc_bond_heartbeat_timeout(pc_bond_heartbeat_timeout),
    _default_ice_servers(default_ice_servers),
    _queue_sizes(queue_sizes),
    _srv(*this),
    _auto_close_media(false) {
}

Host::Host(const Host& other) :
    _nh(other._nh),
    _video_srcs(other._video_srcs),
    _audio_src(other._audio_src),
    _pc_constraints(other._pc_constraints),
    _pc_bond_connect_timeout(other._pc_bond_connect_timeout),
    _pc_bond_heartbeat_timeout(other._pc_bond_heartbeat_timeout),
    _default_ice_servers(other._default_ice_servers),
    _ice_servers(other._ice_servers),
    _queue_sizes(other._queue_sizes),
    _srv(*this),
    _auto_close_media(false) {
}

Host::~Host() {
    close();
}

bool Host::open(bool media) {
    if (!_create_pc_factory()) {
        close();
        return false;
    }
    if (media) {
        ROS_INFO_STREAM("opening media sources");
        if (!_open_media()) {
            close();
            return false;
        }
    } else {
        ROS_DEBUG_STREAM("deferring media sources");
    }
    _srv.advertise();
    return true;
}

bool Host::is_open() const {
    return _pc_factory.get() != NULL;
}

void Host::close() {
    _close_media();
    _pc_factory = NULL;
    _worker_thd.reset();
    _signaling_thd.reset();
    _srv.shutdown();
}

PeerConnectionPtr Host::create_peer_connection(
    const std::string& node_name,
    const std::string& session_id,
    const std::string& peer_id,
    const MediaConstraints& sdp_constraints,
    const std::vector<std::string>& audio_sources,
    const std::vector<std::string>& video_sources) {
    ROS_INFO(
        "creating peer connection id='%s', peer='%s' for node='%s'",
        session_id.c_str(), peer_id.c_str(), node_name.c_str()
    );

    if (!_is_media_open()) {
        _open_media();
        _auto_close_media = true;
    }

    // key
    PeerConnectionKey key = {session_id, peer_id};
    if (_pcs.find(key) != _pcs.end()) {
        ROS_ERROR("peer connection w/ id='%s', peer='%s' already exists", session_id.c_str(), peer_id.c_str());
        return PeerConnectionPtr();
    }

    // audio sources
    std::vector<PeerConnection::AudioSource> audio_srcs;
    for (auto i = audio_sources.begin(); i != audio_sources.end(); i += 1) {
        if (*(i) == _audio_src.label || *(i) == "*") {
            audio_srcs.push_back(PeerConnection::AudioSource(
                _audio_src.label,
                _audio_src.interface,
                _audio_src.publish
            ));
            break;
        }
    }

    // video sources
    std::vector<PeerConnection::VideoSource> video_srcs;
    for (auto i = _video_srcs.begin(); i != _video_srcs.end(); i++) {
        auto &video_src = (*i);
        for (auto j = video_sources.begin(); j != video_sources.end(); j += 1) {
            if (*(j) == video_src.label || *(j) == "*") {
                video_srcs.push_back(PeerConnection::VideoSource(
                    video_src.label,
                    video_src.interface,
                    video_src.publish
                ));
                break;
            }
        }
    }

    // create it
    PeerConnectionPtr pc(new PeerConnection(
        node_name,
        session_id,
        peer_id,
        sdp_constraints,
        _queue_sizes,
        _pc_bond_connect_timeout,
        _pc_bond_heartbeat_timeout
    ));

    // and start it
    PeerConnection::ObserverPtr pc_observer(new PeerConnectionObserver(*this, pc));
    if (!pc->begin(
        _pc_factory,
        &_pc_constraints,
        _ice_servers,
        audio_srcs,
        video_srcs,
        pc_observer)) {
        return PeerConnectionPtr();
    }

    // register it w/ key
    _pcs[key] = pc;
    return pc;
}

bool Host::delete_peer_connection(const std::string& session_id, const std::string& peer_id) {
    PeerConnectionKey key = {session_id, peer_id};
    auto i = _pcs.find(key);
    if (i == _pcs.end()) {
        ROS_INFO("no pc w/ session_id='%s' peer_id='%s'", session_id.c_str(), peer_id.c_str());
    } else {
        ROS_INFO("deleting pc w/ session_id='%s' peer_id='%s'", session_id.c_str(), peer_id.c_str());
        PeerConnectionPtr pc = (*i).second;
        _pcs.erase(i);
        pc->end();

    }
    if (_pcs.empty() && _auto_close_media) {
        _close_media();
    }
    return true;
}

Host::FlushStats Host::flush() {
    FlushStats flush;
    for (auto i = _pcs.begin(); i != _pcs.end(); i++) {
        auto session_flush = (*i).second->flush();
        flush += session_flush;
    }
    return flush;
}

Host::ReapStats Host::reap(double stale_threshold) {
    ReapStats reap;
    double now = ros::Time::now().toSec();
    for (auto i = _pcs.begin(); i != _pcs.end(); i++) {
        PeerConnectionPtr pc = (*i).second;
        if ((pc->is_connecting() || pc->is_disconnected()) &&
            now - pc->last_connection_state_change() > stale_threshold) {
            ROS_INFO_STREAM(
                "scheduling stale (age=" << now - pc->last_connection_state_change() << ") " <<
                "pc('" << pc->session_id() << "','" << pc->peer_id() << "') "
                "for deletion"
            );
            PeerConnectionKey key = {pc->session_id(), pc->peer_id()};
            ros::CallbackInterfacePtr callback(
                new Host::DeletePeerConnectionCallback(*this, key)
            );
            _nh.getCallbackQueue()->addCallback(callback);
            reap.deleted_connections += 1;
        }
    }
    return reap;
}

bool Host::_create_pc_factory() {
    _network_thd = rtc::Thread::CreateWithSocketServer();
    _network_thd->SetName("network_thread", nullptr);
    if (!_network_thd->Start()) {
        ROS_DEBUG_STREAM("network thread failed to start");
        close();
        return false;
    }

    _worker_thd = rtc::Thread::Create();
    _worker_thd->SetName("worker_thread", nullptr);
    if (!_worker_thd->Start()) {
        ROS_DEBUG_STREAM("worker thread failed to start");
        close();
        return false;
    }

    _signaling_thd = rtc::Thread::Create();
    _signaling_thd->SetName("signaling_thread", NULL);
    if (!_signaling_thd->Start()) {
        ROS_DEBUG_STREAM("signaling thread failed to start");
        close();
        return false;
    }

    ROS_INFO_STREAM("creating pc factory");
    _pc_factory =  webrtc::CreatePeerConnectionFactory(
        _network_thd.get(),
        _worker_thd.get(),
        _signaling_thd.get(),
        NULL,
        NULL,
        NULL
    );
    if (!_pc_factory.get()) {
        ROS_DEBUG_STREAM("failed peer-connection factory create");
        close();
        return false;
    }

    return true;
}

bool Host::_open_media() {
    // audio source
    _audio_src.interface = _pc_factory->CreateAudioSource(&_audio_src.constraints);
    if(_audio_src.interface == NULL) {
        ROS_ERROR("cannot create source '%s' for audio device", _audio_src.label.c_str());
        return false;
    }

    // audio track
    std::string audio_label = _audio_src.label;
    if (audio_label.empty()) {
        std::stringstream ss;
        ss << "a" << 0;
        audio_label = ss.str();
    }
    rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track(
        _pc_factory->CreateAudioTrack(audio_label, _audio_src.interface)
    );
    if(audio_track == NULL) {
        ROS_ERROR(
            "cannot create audio track '%s' for source '%s'",
            audio_label.c_str(), audio_label.c_str()
        );
        return false;
    }

    // audio sink
    if (_audio_src.publish) {
        _audio_src.sink = AudioSinkPtr(new AudioSink(
            _nh,
            join_names({"local", audio_label}),
            _queue_sizes.audio,
            audio_track
        ));
    }

    // video sources
    ROS_DEBUG_STREAM("system video capture devices");
    std::vector<WebRTCVideoCaptureDeviceInfo> webrtc_video_capture_devices;
    WebRTCVideoCaptureDeviceInfo::scan(webrtc_video_capture_devices);
    ROS_DEBUG_STREAM("ros video capture topics");
    ROSVideoCaptureTopicsPtr ros_video_capture_topics(new ROSVideoCaptureTopics());
    for (size_t i = 0; i != _video_srcs.size(); i++) {
        VideoSource& video_src = _video_srcs[i];
        if (video_src.type != VideoSource::ROSType)
            continue;
        ros_video_capture_topics->add(video_src.name);
    }
    WebRTCVideoDeviceCapturerFactory webrtc_video_capturer_factory(
        _video_capture_modules
    );
    ROSVideoDeviceCapturerFactory ros_video_capturer_factory(
        _video_capture_modules,
        ros_video_capture_topics
    );
#ifdef USE_MADMUX
    GeoVideoDeviceCapturerFactory geo_video_capturer_factory(
        _video_capture_modules
    );
#endif
    cricket::WebRtcVideoDeviceCapturerFactory *video_capturer_factory = NULL;
    for (size_t i = 0; i != _video_srcs.size(); i++) {
        VideoSource& video_src = _video_srcs[i];

        // capturer
        cricket::VideoDeviceCapturerFactory *video_capturer_factory = NULL;
        cricket::Device device;
        switch (video_src.type) {
            case VideoSource::NameType: {
                auto j = std::find_if(
                    webrtc_video_capture_devices.begin(),
                    webrtc_video_capture_devices.end(),
                    WebRTCVideoCaptureDeviceInfo::find_by_name({video_src.name})
                );
                if (j == webrtc_video_capture_devices.end()) {
                    ROS_ERROR_STREAM(
                        "no webrtc video capture device w/ name \"" << video_src.name << "\""
                    );
                    return false;
                }
                device.id = (*j).unique_id;
                device.name = (*j).name;
                video_capturer_factory = &webrtc_video_capturer_factory;
                break;
            }
#ifdef USE_MADMUX
            case VideoSource::MuxType: {
                auto video0 = webrtc_video_capture_devices.begin();
                device.id = video0->unique_id;
                device.name = video0->name;
                video_capturer_factory = &geo_video_capturer_factory;
                break;
            }
#endif
            case VideoSource::IdType: {
                auto j = std::find_if(
                    webrtc_video_capture_devices.begin(),
                    webrtc_video_capture_devices.end(),
                    WebRTCVideoCaptureDeviceInfo::find_by_file_id({video_src.name})
                );
                if (j == webrtc_video_capture_devices.end()) {
                    ROS_ERROR_STREAM(
                        "no webrtc video capture device w/ file \"" << video_src.name << "\""
                    );
                    return false;
                }
                device.id = (*j).unique_id;
                device.name = (*j).name;
                video_capturer_factory = &webrtc_video_capturer_factory;
                break;
            }
            case VideoSource::ROSType:
                if (ros_video_capture_topics->find(video_src.name) == -1) {
                    ROS_ERROR_STREAM("no id for video src '" << video_src.name << "'");
                    return false;
                }
                device.id = video_src.name;
                device.name = video_src.name;
                video_capturer_factory = &ros_video_capturer_factory;
                break;
            default:
                ROS_ERROR_STREAM(
                    "video src '" << video_src.name <<"' " <<
                    "type '" << video_src.type << "' " <<
                    "not supported."
                );
                return false;
        }
        std::unique_ptr<cricket::VideoCapturer> video_capturer(
            video_capturer_factory->Create(device)
        );
        if (video_capturer.get() == NULL) {
            ROS_ERROR("failed to create video capture device for '%s'", video_src.name.c_str());
            return false;
        }
        video_src.capture_module = _video_capture_modules->find(
            video_capturer->GetId()
        );
        if (!video_src.capture_module) {
            ROS_ERROR_STREAM(
                "video src '" << video_src.name << "' " <<
                "capturer id '" << video_capturer->GetId() << "' not registered"
            );
            return false;
        }

        // source
        video_src.interface = _pc_factory->CreateVideoSource(
            video_capturer.get(), &video_src.constraints
        );
        if(!video_src.interface) {
            ROS_ERROR(
                "cannot create source '%s' for video capture device '%s'",
                video_src.label.c_str(), video_src.name.c_str()
            );
            return false;
        }
        video_capturer.release();

        // rotation
        if (video_src.rotation != 0) {
            webrtc::VideoRotation rotation = webrtc::kVideoRotation_0;
            switch (video_src.rotation) {
            case 0:
                rotation = webrtc::kVideoRotation_0;
                break;
            case 90:
                rotation = webrtc::kVideoRotation_90;
                break;
            case 180:
                rotation = webrtc::kVideoRotation_180;
                break;
            case 270:
                rotation = webrtc::kVideoRotation_270;
                break;
            default:
                ROS_ERROR_STREAM(
                    "invalid rotation " <<
                    video_src.rotation <<
                    ", must be one of 0, 90, 180, 270"
                );
            }
            video_src.capture_module->SetCaptureRotation(rotation);
        }

        // track
        std::string video_label = video_src.label;
        if (video_label.empty()) {
            std::stringstream ss;
            ss << "v" << i + 1;
            video_label = ss.str();
        }
        rtc::scoped_refptr<webrtc::VideoTrackInterface> video_track(
            _pc_factory->CreateVideoTrack(video_label, video_src.interface)
        );
        if(video_track == NULL) {
            ROS_ERROR(
                "cannot create video track '%s' for source '%s'",
                video_label.c_str(), video_label.c_str()
            );
            return false;
        }

        // renderer
        if (video_src.publish) {
            video_src.renderer = VideoRendererPtr(new VideoRenderer(
                _nh,
                join_names({"local", video_label}),
                _queue_sizes.video,
                video_track
            ));
        }
    }

    return true;
}

bool Host::_is_media_open() const {
    if (_audio_src.interface != NULL) {
        return true;
    }
    for (auto i = _video_srcs.begin(); i != _video_srcs.end(); i++) {
        if ((*i).interface != NULL) {
            return true;
        }
    }
    return false;
}

void Host::_close_media() {
    _auto_close_media = false;
    _audio_src.interface = NULL;
    for (auto i = _video_srcs.begin(); i != _video_srcs.end(); i++) {
        if ((*i).renderer) {
            (*i).renderer->Close();
            (*i).renderer = NULL;
        }
        if ((*i).interface) {
            (*i).interface->Stop();
        }
        (*i).interface = NULL;
    }
    _video_capture_modules->remove_all();
}

PeerConnectionPtr Host::_find_peer_connection(const PeerConnectionKey& key) {
    auto i = _pcs.find(key);
    if (i == _pcs.end())
        return PeerConnectionPtr();
    return (*i).second;
}

PeerConnectionConstPtr Host::_find_peer_connection(const PeerConnectionKey& key) const {
    auto i = _pcs.find(key);
    if (i == _pcs.end())
        return PeerConnectionConstPtr();
    return (*i).second;
}

// Host::Flush

Host::FlushStats& Host::FlushStats::operator += (const PeerConnection::FlushStats & rhs) {
    reaped_data_messages += rhs.reaped_data_messages;
    return *this;
}

// Host::Service

Host::Service::Service(Host& instance) : _instance(instance) {
}

void Host::Service::advertise() {
    _srvs.push_back(_instance._nh.advertiseService("add_ice_candidate", &Host::Service::add_ice_candidate, this));
    _srvs.push_back(_instance._nh.advertiseService("create_data_channel", &Host::Service::create_data_channel, this));
    _srvs.push_back(_instance._nh.advertiseService("create_offer", &Host::Service::create_offer, this));
    _srvs.push_back(_instance._nh.advertiseService("create_peer_connection", &Host::Service::create_peer_connection, this));
    _srvs.push_back(_instance._nh.advertiseService("delete_peer_connection", &Host::Service::delete_peer_connection, this));
    _srvs.push_back(_instance._nh.advertiseService("get_host", &Host::Service::get_host, this));
    _srvs.push_back(_instance._nh.advertiseService("get_peer_connection", &Host::Service::get_peer_connection, this));
    _srvs.push_back(_instance._nh.advertiseService("send_data", &Host::Service::send_data, this));
    _srvs.push_back(_instance._nh.advertiseService("set_ice_servers", &Host::Service::set_ice_servers, this));
    _srvs.push_back(_instance._nh.advertiseService("set_remote_description", &Host::Service::set_remote_description, this));
    _srvs.push_back(_instance._nh.advertiseService("rotate_video_source", &Host::Service::rotate_video_source, this));
}

void Host::Service::shutdown() {
    _srvs.clear();
}

bool Host::Service::add_ice_candidate(ros::ServiceEvent<ros_webrtc::AddIceCandidate::Request, ros_webrtc::AddIceCandidate::Response>& event) {
    const auto& req = event.getRequest();
    PeerConnectionKey key = {req.session_id, req.peer_id};
    PeerConnectionPtr pc = _instance._find_peer_connection(key);
    if (pc == NULL)
        return false;
    // FIXME: detect and log error
    std::unique_ptr<webrtc::IceCandidateInterface> ice_candidate(webrtc::CreateIceCandidate(
        req.sdp_mid, req.sdp_mline_index, req.candidate, NULL
    ));
    pc->add_ice_candidate(ice_candidate.get());
    return true;
}

bool Host::Service::create_data_channel(ros::ServiceEvent<ros_webrtc::CreateDataChannel::Request, ros_webrtc::CreateDataChannel::Response>& event) {
    const auto& req = event.getRequest();
    PeerConnectionKey key = {req.session_id, req.peer_id};
    PeerConnectionPtr pc= _instance._find_peer_connection(key);
    if (pc == NULL)
        return false;
    return pc->create_data_channel(
        req.label,
        req.protocol,
        req.reliable,
        req.ordered,
        req.id
    );
}

bool Host::Service::create_offer(ros::ServiceEvent<ros_webrtc::CreateOffer::Request, ros_webrtc::CreateOffer::Response>& event) {
    const auto& req = event.getRequest();
    PeerConnectionKey key = {req.session_id, req.peer_id};
    PeerConnectionPtr pc = _instance._find_peer_connection(key);
    if (pc == NULL)
       return false;
    if (!pc->create_offer())
       return false;
   return true;
}

bool Host::Service::create_peer_connection(ros::ServiceEvent<ros_webrtc::CreatePeerConnection::Request, ros_webrtc::CreatePeerConnection::Response>& event) {
    const auto &req = event.getRequest();

    if (!_instance._is_media_open()) {
        if (!_instance._open_media()) {
            return false;
        }
        _instance._auto_close_media = true;
    }

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

    PeerConnectionPtr pc(_instance.create_peer_connection(
        event.getCallerName(),
        req.session_id,
        req.peer_id,
        sdp_constraints,
        req.audio_sources,
        req.video_sources
    ));
    return pc != NULL;
}

bool Host::Service::delete_peer_connection(ros::ServiceEvent<ros_webrtc::DeletePeerConnection::Request, ros_webrtc::DeletePeerConnection::Response>& event) {
    const auto& req = event.getRequest();
    return _instance.delete_peer_connection(req.session_id, req.peer_id);
}

bool Host::Service::get_host(ros::ServiceEvent<ros_webrtc::GetHost::Request, ros_webrtc::GetHost::Response>& event) {
    const auto &req = event.getRequest();
    auto &resp = event.getResponse();

    // peer connection constraints
    resp.sdp_constraints = _instance._pc_constraints;

    // audio sources
    const auto &audio_src = _instance._audio_src;
    ros_webrtc::Source src;
    src.kind = "audio";
    src.label = audio_src.label;
    src.state = audio_src.interface != NULL ? to_string(audio_src.interface->state()) : "ended";
    src.publish = audio_src.publish;
    resp.audio_sources.push_back(src);

    // video sources
    for (auto i = _instance._video_srcs.begin(); i != _instance._video_srcs.end(); i++) {
        const auto &video_src = *(i);

        ros_webrtc::Source src;
        src.kind = "video";
        src.label = video_src.label;
        src.state = video_src.interface != NULL ? to_string(video_src.interface->state()) : "ended";
        src.publish = video_src.publish;
        src.rotation = video_src.rotation;

        resp.video_sources.push_back(src);
    }

    // peer connections
    for (auto i = _instance._pcs.begin(); i != _instance._pcs.end(); i++) {
        ros_webrtc::PeerConnectionKey key;
        key.session_id = (*i).second->session_id();
        key.peer_id = (*i).second->peer_id();
        resp.peer_connections.push_back(key);
    }

    return true;
}

bool Host::Service::get_peer_connection(ros::ServiceEvent<ros_webrtc::GetPeerConnection::Request, ros_webrtc::GetPeerConnection::Response>& event) {
    const auto &req = event.getRequest();
    PeerConnectionKey key = {req.session_id, req.peer_id};
    PeerConnectionPtr pc = _instance._find_peer_connection(key);
    if (pc == NULL) {
        ROS_INFO_STREAM(
            "pc (" << key.session_id << "', '" << key.peer_id << "') " <<
            "not found"
        );
        return false;
    }
    auto &resp = event.getResponse();
    resp.peer_connection = *pc;
    return true;
}

bool Host::Service::send_data(ros::ServiceEvent<ros_webrtc::SendData::Request, ros_webrtc::SendData::Response>& event) {
    const auto& req = event.getRequest();
    PeerConnectionKey key = {req.session_id, req.peer_id};
    PeerConnectionPtr pc = _instance._find_peer_connection(key);
    if (pc == NULL) {
        ROS_INFO_STREAM(
            "pc (" << key.session_id << "', '" << key.peer_id << "') " <<
            "not found"
        );
        return false;
    }
    DataChannelPtr dc = pc->data_channel(req.data.label);
    if (dc == NULL) {
        ROS_INFO_STREAM(
            "pc (" << key.session_id << "', '" << key.peer_id << "') " <<
            "has no data channel w/ label " << req.data.label
        );
        return false;
    }
    dc->send(req.data);
    return true;
}

bool Host::Service::set_ice_servers(
        ros::ServiceEvent<ros_webrtc::SetIceServers::Request,
        ros_webrtc::SetIceServers::Response>& event) {
    const auto& req = event.getRequest();
    _instance._ice_servers = _instance._default_ice_servers;

    for (auto i = 0; i < req.ice_servers.size(); i++) {
        webrtc::PeerConnectionInterface::IceServer server;
        server.uri = req.ice_servers[i].uri;
        server.username = req.ice_servers[i].username;
        server.password = req.ice_servers[i].password;
        _instance._ice_servers.push_back(server);

    }
    return true;
}

bool Host::Service::set_remote_description(ros::ServiceEvent<ros_webrtc::SetRemoteDescription::Request, ros_webrtc::SetRemoteDescription::Response>& event) {
    const auto& req = event.getRequest();
    PeerConnectionKey key = {req.session_id, req.peer_id};
    PeerConnectionPtr pc = _instance._find_peer_connection(key);
    if (pc == NULL) {
        ROS_INFO_STREAM(
            "pc (" << key.session_id << "', '" << key.peer_id << "') " <<
            "not found"
        );
        return false;
    }
    webrtc::SdpParseError err;
    auto desc = webrtc::CreateSessionDescription(
        req.session_description.type,
        req.session_description.sdp,
        &err
    );
    if (desc == NULL) {
        ROS_INFO(
            "webrtc::CreateSessionDescription() == NULL - line=%s, description=%s",
            err.line.c_str(), err.description.c_str()
        );
        return false;
    }
    pc->set_remote_session_description(desc);
    if (!pc->is_offerer()) {
        pc->create_answer();
    }
    return true;
}

bool Host::Service::rotate_video_source(ros::ServiceEvent<ros_webrtc::RotateVideoSource::Request, ros_webrtc::RotateVideoSource::Response>& event) {
    const auto& req = event.getRequest();

    // src
    VideoSource video_src;
    for (auto i = _instance._video_srcs.begin(); i != _instance._video_srcs.end(); i++) {
        if ((*i).label== req.label) {
            video_src = (*i);
        }
    }
    if (video_src.label.empty()) {
        ROS_ERROR_STREAM("no video src w/ label '" << req.label << "'.");
        return false;
    }

    // capture module
    rtc::scoped_refptr<webrtc::VideoCaptureModule> capture_module = video_src.capture_module;
    if (!capture_module) {
        ROS_ERROR_STREAM(
            "video src '" << video_src.name << "' " << "capture module not registered"
        );
        return false;
    }
    webrtc::VideoRotation rotation;
    switch (req.rotation) {
    case 0:
        rotation = webrtc::kVideoRotation_0;
        break;
    case 90:
        rotation = webrtc::kVideoRotation_90;
        break;
    case 180:
        rotation = webrtc::kVideoRotation_180;
        break;
    case 270:
        rotation = webrtc::kVideoRotation_270;
        break;
    default:
        ROS_ERROR_STREAM(
            "invalid rotation " <<
            req.rotation <<
            ", must be one of 0, 90, 180, 270"
        );
        return false;
    }
    capture_module->SetCaptureRotation(rotation);
    ROS_INFO_STREAM(
        "video src '" << video_src.name << "' rotation set to " << req.rotation
    );
    video_src.rotation = rotation;
    return true;
}

// Host::DeletePeerConnectionCallback

Host::DeletePeerConnectionCallback::DeletePeerConnectionCallback(Host& instance, const PeerConnectionKey& key) :
    _instance(instance),
    _key(key) {
}

ros::CallbackInterface::CallResult Host::DeletePeerConnectionCallback::call() {
    _instance.delete_peer_connection(_key.session_id, _key.peer_id);
    return Success;
}


// Host::PeerConnectionObserver

Host::PeerConnectionObserver::PeerConnectionObserver(Host& instance, PeerConnectionPtr pc) :
    _instance(instance),
    _pc(pc) {
}

Host::PeerConnectionObserver::~PeerConnectionObserver() {
}

void Host::PeerConnectionObserver::on_connection_change(webrtc::PeerConnectionInterface::IceConnectionState state) {
    if (state == webrtc::PeerConnectionInterface::kIceConnectionDisconnected) {
        PeerConnectionKey key = {_pc->session_id(), _pc->peer_id()};
        ros::CallbackInterfacePtr callback(new Host::DeletePeerConnectionCallback(_instance, key));
        _instance._nh.getCallbackQueue()->addCallback(callback);
    }
}

void Host::PeerConnectionObserver::on_bond_broken() {
    PeerConnectionKey key = {_pc->session_id(), _pc->peer_id()};
    ros::CallbackInterfacePtr callback(new Host::DeletePeerConnectionCallback(_instance, key));
    _instance._nh.getCallbackQueue()->addCallback(callback);
}

// HostFactory

Host HostFactory::operator()(ros::NodeHandle &nh) {
    return Host(
        nh,
        video_srcs,
        audio_src,
        pc_constraints,
        pc_bond_connect_timeout,
        pc_bond_heartbeat_timeout,
        default_ice_servers,
        queue_sizes
    );
}
