#include "host.h"

#include <talk/app/webrtc/peerconnectioninterface.h>
#include <talk/app/webrtc/videosourceinterface.h>
#include <talk/media/devices/devicemanager.h>
#include <talk/media/webrtc/webrtcvideoencoderfactory.h>
#include <talk/media/webrtc/webrtcvideodecoderfactory.h>

#include "convert.h"
#include "device_manager.h"
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
    double pc_bond_timeout,
    const std::vector<webrtc::PeerConnectionInterface::IceServer>& ice_servers,
    const QueueSizes& queue_sizes) :
    _nh(nh),
    _video_srcs(video_srcs),
    _video_capture_modules(new VideoCaptureModuleRegistry()),
    _audio_src(audio_src),
    _pc_constraints(pc_constraints),
    _pc_bond_timeout(pc_bond_timeout),
    _ice_servers(ice_servers),
    _queue_sizes(queue_sizes),
    _srv(*this),
    _auto_close_media(false) {
}

Host::Host(const Host& other) :
    _nh(other._nh),
    _video_srcs(other._video_srcs),
    _audio_src(other._audio_src),
    _pc_constraints(other._pc_constraints),
    _pc_bond_timeout(other._pc_bond_timeout),
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
        _pc_bond_timeout
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

bool Host::_create_pc_factory() {
    ROS_INFO_STREAM("creating worker thread");
    _worker_thd.reset(new rtc::Thread());
    _worker_thd->SetName("worker_thread", NULL);
    if (!_worker_thd->Start()) {
        ROS_DEBUG_STREAM("worker thread failed to start");
        close();
        return false;
    }

    ROS_INFO_STREAM("creating signaling thread");
    _signaling_thd.reset(new rtc::Thread());
    _signaling_thd->SetName("signaling_thread", NULL);
    if (!_signaling_thd->Start()) {
        ROS_DEBUG_STREAM("signaling thread failed to start");
        close();
        return false;
    }

    ROS_INFO_STREAM("creating pc factory");
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

bool Host::_open_media() {
    ROS_DEBUG_STREAM("creating system device manager");
    rtc::scoped_ptr<cricket::DeviceManagerInterface> default_dev_mgr(
        cricket::DeviceManagerFactory::Create()
    );
    if (!default_dev_mgr->Init()) {
        ROS_ERROR_STREAM("cannot create default device manager");
        return false;
    }
    default_dev_mgr->SetVideoDeviceCapturerFactory(
        new WebRtcVideoDeviceCapturerFactory(_video_capture_modules)
    );

    ROS_DEBUG_STREAM("creating ros device manager");
    ROSVideoCaptureTopicInfoPtr video_capture_topics(new ROSVideoCaptureTopicInfo());
    for (size_t i = 0; i != _video_srcs.size(); i++) {
        VideoSource& video_src = _video_srcs[i];
        if (video_src.type != VideoSource::ROSType)
            continue;
        video_capture_topics->add(video_src.name);
    }
    rtc::scoped_ptr<cricket::DeviceManagerInterface> ros_dev_mgr(
        new ROSDeviceManager(video_capture_topics, _video_capture_modules)
    );
    if (!ros_dev_mgr->Init()) {
        ROS_ERROR_STREAM("cannot create ros device manager");
        return false;
    }

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
    for (size_t i = 0; i != _video_srcs.size(); i++) {
        VideoSource& video_src = _video_srcs[i];

        // device manager
        cricket::DeviceManagerInterface *dev_mgr = NULL;
        switch (video_src.type) {
            case VideoSource::NameType:
            case VideoSource::IdType:
                dev_mgr = default_dev_mgr.get();
                break;
            case VideoSource::ROSType:
                dev_mgr = ros_dev_mgr.get();
                break;
            default:
                ROS_ERROR_STREAM(
                    "video src '" << video_src.name <<
                    "' type '" << video_src.type <<
                    "' not supported."
                );
                return false;
        }

        // capturer
        cricket::Device device;
        if (video_src.type == VideoSource::IdType) {
            std::vector<cricket::Device> devs;
            if (!dev_mgr->GetVideoCaptureDevices(&devs)) {
                ROS_ERROR("cannot get video capture devices");
                return false;
            }
            for (auto i = devs.begin(); i != devs.end(); i++) {
                if ((*i).id == video_src.name) {
                    device = *i;
                    break;
                }
            }
            if (device.name.empty()) {
                ROS_ERROR("cannot get video capture device for file '%s'", video_src.name.c_str());
                return false;
            }
        } else if (!dev_mgr->GetVideoCaptureDevice(video_src.name, &device)) {
            ROS_ERROR("cannot get video capture device for '%s'", video_src.name.c_str());
            return false;
        }
        std::auto_ptr<cricket::VideoCapturer> video_capturer(dev_mgr->CreateVideoCapturer(device));
        if (video_capturer.get() == NULL) {
            ROS_ERROR("failed to create video capture device for '%s'", video_src.name.c_str());
            return false;
        }

        // source
        video_src.interface = _pc_factory->CreateVideoSource(video_capturer.get(), &video_src.constraints);
        if(video_src.interface.get() == NULL) {
            ROS_ERROR(
                "cannot create source '%s' for video capture device '%s'",
                video_src.label.c_str(), video_src.name.c_str()
            );
            return false;
        }
        video_capturer.release();

        // rotation
        if (video_src.rotation != 0) {
            rtc::scoped_refptr<webrtc::VideoCaptureModule> vcm(
                _video_capture_modules->find(
                   video_src.interface->GetVideoCapturer()->GetId()
                )
            );
            if (vcm == NULL) {
                ROS_ERROR_STREAM(
                    "video src '" <<
                    video_src.name <<
                    "' capturer id '" <<
                    video_src.interface->GetVideoCapturer()->GetId() <<
                    "' not registered"
                );
            } else {
                webrtc::VideoCaptureRotation rotation = webrtc::kCameraRotate0;
                switch (video_src.rotation) {
                case 0:
                    rotation = webrtc::kCameraRotate0;
                    break;
                case 90:
                    rotation = webrtc::kCameraRotate90;
                    break;
                case 180:
                    rotation = webrtc::kCameraRotate180;
                    break;
                case 270:
                    rotation = webrtc::kCameraRotate270;
                    break;
                default:
                    ROS_ERROR_STREAM(
                        "invalid rotation " <<
                        video_src.rotation <<
                        ", must be one of 0, 90, 180, 270"
                    );
                }
                vcm->SetCaptureRotation(rotation);
            }
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
    _video_capture_modules->remove_all();
    _auto_close_media = false;
    _audio_src.interface = NULL;
    for (auto i = _video_srcs.begin(); i != _video_srcs.end(); i++) {
        (*i).renderer = NULL;
        (*i).interface = NULL;
    }
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
    rtc::scoped_ptr<webrtc::IceCandidateInterface> ice_candidate(webrtc::CreateIceCandidate(
        req.sdp_mid, req.sdp_mline_index, req.candidate
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
    if (pc == NULL)
        return false;
    auto &resp = event.getResponse();
    resp.peer_connection = *pc;
    return true;
}

bool Host::Service::send_data(ros::ServiceEvent<ros_webrtc::SendData::Request, ros_webrtc::SendData::Response>& event) {
    const auto& req = event.getRequest();
    PeerConnectionKey key = {req.session_id, req.peer_id};
    PeerConnectionPtr pc = _instance._find_peer_connection(key);
    if (pc == NULL)
        return false;
    DataChannelPtr dc = pc->data_channel(req.data.label);
    if (dc == NULL)
        return false;
    dc->send(req.data);
    return true;
}

bool Host::Service::set_remote_description(ros::ServiceEvent<ros_webrtc::SetRemoteDescription::Request, ros_webrtc::SetRemoteDescription::Response>& event) {
    const auto& req = event.getRequest();
    PeerConnectionKey key = {req.session_id, req.peer_id};
    PeerConnectionPtr pc = _instance._find_peer_connection(key);
    if (pc == NULL)
        return false;
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
    rtc::scoped_refptr<webrtc::VideoCaptureModule> vcm(
        _instance._video_capture_modules->find(
           video_src.interface->GetVideoCapturer()->GetId()
        )
    );
    if (vcm == NULL) {
        ROS_ERROR_STREAM(
            "video src '" <<
            video_src.name <<
            "' capturer id '" <<
            video_src.interface->GetVideoCapturer()->GetId() <<
            "' not registered"
        );
        return false;
    }
    webrtc::VideoCaptureRotation rotation;
    switch (req.rotation) {
    case 0:
        rotation = webrtc::kCameraRotate0;
        break;
    case 90:
        rotation = webrtc::kCameraRotate90;
        break;
    case 180:
        rotation = webrtc::kCameraRotate180;
        break;
    case 270:
        rotation = webrtc::kCameraRotate270;
        break;
    default:
        ROS_ERROR_STREAM(
            "invalid rotation " <<
            req.rotation <<
            ", must be one of 0, 90, 180, 270"
        );
        return false;
    }
    vcm->SetCaptureRotation(rotation);
    ROS_INFO_STREAM(
        "video src '" <<
        video_src.name <<
        "' capturer id '" <<
        video_src.interface->GetVideoCapturer()->GetId() <<
        "' rotation set to " << req.rotation
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
        pc_bond_timeout,
        ice_servers,
        queue_sizes
    );
}
