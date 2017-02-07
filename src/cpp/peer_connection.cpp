#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <cpp/peer_connection.h>
#include <ros/ros.h>
#include <ros_webrtc/Close.h>
#include <ros_webrtc/DataChannel.h>
#include <ros_webrtc/IceCandidate.h>
#include <ros_webrtc/IceConnectionState.h>
#include <ros_webrtc/SessionDescription.h>
#include <ros_webrtc/SignalingState.h>
#include <ros_webrtc/Stream.h>
#include <ros_webrtc/OnAddStream.h>
#include <ros_webrtc/OnDataChannel.h>
#include <ros_webrtc/OnIceCandidate.h>
#include <ros_webrtc/OnIceConnectionStateChange.h>
#include <ros_webrtc/OnNegotiationNeeded.h>
#include <ros_webrtc/OnRemoveStream.h>
#include <ros_webrtc/OnSetSessionDescription.h>
#include <ros_webrtc/OnSignalingStateChange.h>
#include <std_msgs/Empty.h>
#include <webrtc/api/jsepicecandidate.h>

#include "convert.h"
#include "host.h"
#include "util.h"

#include "peer_connection.h"


// PeerConnection

PeerConnection::PeerConnection(
    const std::string& node_name,
    const std::string& session_id,
    const std::string& peer_id,
    const MediaConstraints& sdp_constraints,
    const QueueSizes& queue_sizes,
    double connect_timeout,
    double heartbeat_timeout) :
    _nn(node_name),
    _session_id(session_id),
    _peer_id(peer_id),
    _ice_connection_changed_at(ros::Time::now().toSec()),
    _sdp_constraints(sdp_constraints),
    _pco(*this),
    _csdo(new PeerConnection::CreateSessionDescriptionObserver(*this)),
    _ssdo(new PeerConnection::SetSessionDescriptionObserver(*this)),
    _is_offerer(false),
    _queue_remote_ice_candidates(true),
    _events(*this),
    _callbacks(*this),
    _queue_sizes(queue_sizes),
    _bond(
        "peer_connection_bond",
        _session_id + "_" + _peer_id,
        boost::function<void (void)>(boost::bind(&PeerConnection::_on_bond_broken, this)),
        boost::function<void (void)>(boost::bind(&PeerConnection::_on_bond_formed, this))
    ) {
    _bond.setConnectTimeout(connect_timeout);
    _bond.setHeartbeatTimeout(heartbeat_timeout);
    ROS_INFO_STREAM(
        "pc ('" << _session_id << "', '"<< _peer_id << "') bond - " <<
        "connect_timeout=" << _bond.getConnectTimeout() << ", " <<
        "heartbeat_timeout=" << _bond.getHeartbeatTimeout()
    );
}

const std::string& PeerConnection::session_id() const {
    return _session_id;
}

const std::string& PeerConnection::peer_id() const {
    return _peer_id;
}

bool PeerConnection::is_connecting() const {
    if (_pc == nullptr) {
        return false;
    }
    auto state = _pc->ice_connection_state();
    return (
        state == webrtc::PeerConnectionInterface::kIceConnectionNew ||
        state == webrtc::PeerConnectionInterface::kIceConnectionChecking
    );
}


bool PeerConnection::is_disconnected() const {
    if (_pc == nullptr) {
        return false;
    }
    auto state = _pc->ice_connection_state();
    return (
        state == webrtc::PeerConnectionInterface::kIceConnectionFailed ||
        state == webrtc::PeerConnectionInterface::kIceConnectionDisconnected ||
        state == webrtc::PeerConnectionInterface::kIceConnectionClosed
    );
}

double PeerConnection::last_connection_state_change() const {
    return _ice_connection_changed_at;
}

std::string PeerConnection::topic(const std::string &name) const {
    return join_names({
        "session_" + _session_id,
        "peer_" + _peer_id,
        name
    });
}

std::string PeerConnection::callback(const std::string &name) const {
    return join_names({
        "/" + _nn,
        "session_" + _session_id,
        "peer_" + _peer_id,
        name
    });
}

bool PeerConnection::begin(
    webrtc::PeerConnectionFactoryInterface* pc_factory,
    const webrtc::MediaConstraintsInterface* pc_constraints,
    const webrtc::PeerConnectionInterface::IceServers& ice_servers,
    const std::vector<AudioSource> &audio_srcs,
    const std::vector<VideoSource> &video_srcs,
    PeerConnection::ObserverPtr observer) {
    _observer = observer;
    if (!_open_local_stream(pc_factory, audio_srcs, video_srcs)) {
        end();
        return false;
    }
    if (!_open_peer_connection(pc_factory, pc_constraints, ice_servers)) {
        end();
        return false;
    }
    if (_bond.getHeartbeatTimeout() != 0) {
        ROS_INFO_STREAM(
            "pc ('" << _session_id << "', '"<< _peer_id << "') bond forming"
        );
        _bond.start();
    } else {
        ROS_INFO_STREAM(
            "pc ('" << _session_id << "', '"<< _peer_id << "') bond disabled"
        );
    }
    return true;
}

void PeerConnection::end() {
    _close_peer_connection();
    _observer.reset();
    _events.shutdown();
    _callbacks.shutdown();
    if (_bond.getHeartbeatTimeout() != 0) {
        _bond.breakBond();
    }
}

webrtc::PeerConnectionInterface* PeerConnection::peer_connection() {
    return _pc.get();
}

bool PeerConnection::create_data_channel(
        std::string label,
        std::string protocol,
        bool reliable,
        bool ordered,
        int id) {
    webrtc::DataChannelInit init;
    init.id = id;
    init.protocol = protocol;
    init.ordered = ordered;
    init.reliable = reliable;
    rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel = _pc->CreateDataChannel(label, &init);
    if (data_channel == NULL) {
        ROS_ERROR_STREAM(
            "pc('" << _session_id << "', '" << _peer_id << "') " <<
            "does not support data"
        );
        return false;
    }
    MediaType media_type;
    if (!data_channel->protocol().empty()) {
        if (!MediaType::matches(data_channel->protocol())) {
            ROS_WARN_STREAM(
                "pc('" << _session_id << "', '"<< _peer_id << "') " <<
                "data channel w/ label='" << data_channel->label() << "'" <<
                "has non media-type protocol='" << data_channel->protocol() << "'.";
            );
        } else {
            media_type = MediaType(data_channel->protocol());
        }
    }
    DataChannelPtr dc(new DataChannel(
        _nh,
        topic("data_" + label),
        data_channel,
        media_type,
        _queue_sizes.data
    ));
    _dcs[label] = dc;
    return true;
}

DataChannelPtr PeerConnection::data_channel(const std::string& label) {
    auto i = _dcs.find(label);
    if (i == _dcs.end())
        return DataChannelPtr();
    return (*i).second;
}

DataChannelConstPtr PeerConnection::data_channel(const std::string& label) const {
    auto i = _dcs.find(label);
    if (i == _dcs.end())
        return DataChannelConstPtr();
    return (*i).second;
}

bool PeerConnection::create_offer() {
    _is_offerer = true;
    _pc->CreateOffer(_csdo, &_sdp_constraints);
    return true;
}

void PeerConnection::create_answer() {
    _is_offerer = false;
    _pc->CreateAnswer(_csdo, &_sdp_constraints);
}

bool PeerConnection::is_offerer() const {
    return _is_offerer;
}

void PeerConnection::add_ice_candidate(webrtc::IceCandidateInterface* candidate) {
    if (_queue_remote_ice_candidates) {
        // FIXME: capture and log error
        std::string sdp;
        candidate->ToString(&sdp);
        IceCandidatePtr copy(webrtc::CreateIceCandidate(
            candidate->sdp_mid(),
            candidate->sdp_mline_index(),
            sdp,
            NULL
        ));
        _remote_ice_cadidates.push_back(copy);
    } else {
        _pc->AddIceCandidate(candidate);
    }
}

void PeerConnection::set_remote_session_description(webrtc::SessionDescriptionInterface* sdp) {
    _pc->SetRemoteDescription(_ssdo, sdp);
}

PeerConnection::operator ros_webrtc::PeerConnection () const {
    ros_webrtc::PeerConnection pc;

    pc.session_id = _session_id;

    pc.peer_id = _peer_id;

    pc.is_offerer = _is_offerer;

    pc.sdp_constraints = _sdp_constraints;

    if (_pc.get() != NULL) {
        // local stream
        auto local_streams = _pc->local_streams();
        for (size_t i = 0; i < local_streams->count(); i+= 1) {
            auto stream = local_streams->at(i);

            // audoio tracks
            auto a_tracks = stream->GetAudioTracks();
            for (auto iter = a_tracks.begin(); iter != a_tracks.end(); iter++) {
                auto & track = *iter;
                pc.local_tracks.push_back(to_ros(track));
            }

            // video tracks
            auto v_tracks = stream->GetVideoTracks();
            for (auto iter = v_tracks.begin(); iter != v_tracks.end(); iter++) {
                auto & track = *iter;
                pc.local_tracks.push_back(to_ros(track));
            }
        }

        // local stream
        auto remote_streams = _pc->remote_streams();
        for (size_t i = 0; i < remote_streams->count(); i+= 1) {
            auto stream = remote_streams->at(i);

            // audoio tracks
            auto a_tracks = stream->GetAudioTracks();
            for (auto iter = a_tracks.begin(); iter != a_tracks.end(); iter++) {
                auto & track = *iter;
                pc.remote_tracks.push_back(to_ros(track));
            }

            // video tracks
            auto v_tracks = stream->GetVideoTracks();
            for (auto iter = v_tracks.begin(); iter != v_tracks.end(); iter++) {
                auto & track = *iter;
                pc.remote_tracks.push_back(to_ros(track));
            }
        }

        pc.signaling_state = to_string(_pc->signaling_state());

        pc.ice_connection_state = to_string(_pc->ice_connection_state());

        pc.ice_gathering_state = to_string(_pc->ice_gathering_state());
    }

    for (auto i = _dcs.begin(); i != _dcs.end(); i++) {
        pc.data_channels.push_back(*(*i).second);
    }

    return pc;
}

PeerConnection::FlushStats PeerConnection::flush() {
    PeerConnection::FlushStats flush;
    for (auto i = _dcs.begin(); i != _dcs.end(); i++) {
        flush.reaped_data_messages += (*i).second->reap();
    }
    return flush;
}

bool PeerConnection::_open_peer_connection(
        webrtc::PeerConnectionFactoryInterface* pc_factory,
        const webrtc::MediaConstraintsInterface* pc_constraints,
        const webrtc::PeerConnectionInterface::IceServers& ice_servers) {
    webrtc::PeerConnectionInterface::RTCConfiguration rtc_conf;
    rtc_conf.servers = ice_servers;
    _pc = pc_factory->CreatePeerConnection(
        rtc_conf, pc_constraints, NULL, NULL, &_pco
    );
    if (_pc == NULL) {
        return false;
    }
    if (!_pc->AddStream(_local_stream)) {
        return false;
    }
    return true;
}

void PeerConnection::_close_peer_connection() {
    _close_local_stream();

    _dcs.clear();

    if (_pc != NULL) {
        _pc->Close();
        _pc = NULL;
    }

    ROS_INFO_STREAM("pc ('" << _session_id << "', '"<< _peer_id << "') closed");

    // event
    ros_webrtc::Close msg;
    msg.session_id = _session_id;
    msg.peer_id = _peer_id;
    _events.on_close.publish(msg);
}

bool PeerConnection::_open_local_stream(
        webrtc::PeerConnectionFactoryInterface* pc_factory,
        const std::vector<AudioSource> &audio_srcs,
        const std::vector<VideoSource> &video_srcs
    ) {
    rtc::scoped_refptr<webrtc::MediaStreamInterface> local_stream;

    // stream
    std::stringstream ss;
    ss << "s" << 1;
    std::string stream_label = ss.str();
    ss.str("");
    ss.clear();
    local_stream = pc_factory->CreateLocalMediaStream(stream_label);

    // audio tracks
    for (size_t i = 0; i != audio_srcs.size(); i++) {
        const auto& audio_src = audio_srcs[i];

        // track
        std::string audio_label = audio_src.label;
        if (audio_label.empty()) {
            ss << "a" << i + 1;
            audio_label = ss.str();
            ss.str("");
            ss.clear();
        }
        rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track(
            pc_factory->CreateAudioTrack(audio_label, audio_src.interface)
        );
        if(audio_track.get() == NULL) {
            ROS_ERROR(
                "cannot create audio track '%s' for source '%s'",
                audio_label.c_str(), audio_src.label.c_str()
            );
            return false;
        }

        local_stream->AddTrack(audio_track);
    }

    // video tracks
    for (size_t i = 0; i != video_srcs.size(); i++) {
        const auto& video_src = video_srcs[i];

        // track
        std::string video_label = video_src.label;
        if (video_label.empty()) {
            ss << "v" << i + 1;
            video_label = ss.str();
            ss.str("");
            ss.clear();
        }
        rtc::scoped_refptr<webrtc::VideoTrackInterface> video_track(
            pc_factory->CreateVideoTrack(video_label, video_src.interface)
        );
        if(video_track.get() == NULL) {
            ROS_ERROR(
                "cannot create video track '%s' for source '%s'",
                video_label.c_str(), video_src.label.c_str()
            );
            return false;
        }

        local_stream->AddTrack(video_track);
    }

    _local_stream = local_stream;

    return true;
}

void PeerConnection::_close_local_stream() {
    while (!_audio_sinks.empty()) {
        _audio_sinks.pop_front();
    }
    while (!_video_renderers.empty()) {
        _video_renderers.pop_front();
    }
    _local_stream = NULL;
}

void PeerConnection::_on_bond_formed() {
    ROS_INFO_STREAM(
        "pc ('" << _session_id << "', '"<< _peer_id << "') bond formed"
    );
}

void PeerConnection::_on_bond_broken() {
    ROS_INFO_STREAM(
        "pc ('" << _session_id << "', '"<< _peer_id << "') bond broken"
    );
    if (_observer != NULL)
        _observer->on_bond_broken();
}

void PeerConnection::_on_local_description(webrtc::SessionDescriptionInterface* desc) {
    ROS_INFO("pc '%s' local description", _session_id.c_str());
    ros_webrtc::SessionDescription msg;
    desc->ToString(&msg.sdp);
    msg.type = is_offerer() ? "offer" : "answer";

    // callback
    if (_callbacks.on_set_session_description.exists()) {
        ros_webrtc::OnSetSessionDescription srv;
        srv.request.type = msg.type;
        srv.request.sdp = msg.sdp;
        _callbacks.on_set_session_description.call(srv);
    }

    // event
    _events.on_set_session_description.publish(msg);
}

void PeerConnection::_drain_remote_ice_candidates() {
    ROS_INFO("pc '%s' adding %zu q'd remote ice candidates", _session_id.c_str(), _remote_ice_cadidates.size());
    while (!_remote_ice_cadidates.empty()) {
        IceCandidatePtr ice_candidate = _remote_ice_cadidates.front();
        _remote_ice_cadidates.pop_front();
        _pc->AddIceCandidate(ice_candidate.get());
    }
    _queue_remote_ice_candidates = false;
}

// PeerConnection::VideoSource

PeerConnection::VideoSource::VideoSource(
    const std::string& label,
    webrtc::VideoTrackSourceInterface *interface,
    bool publish) :
    label(label),
    interface(interface),
    publish(publish) {
}

// PeerConnection::AudioSource

PeerConnection::AudioSource::AudioSource(
    const std::string& label,
    webrtc::AudioSourceInterface *interface,
    bool publish) :
    label(label),
    interface(interface),
    publish(publish){
}

// PeerConnection::PeerConnectionObserver

PeerConnection::PeerConnectionObserver::PeerConnectionObserver(PeerConnection& instance_) : instance(instance_) {
}

void PeerConnection::PeerConnectionObserver::OnSignalingChange(webrtc::PeerConnectionInterface::SignalingState new_state) {
    ROS_INFO_STREAM("pc ('" << instance._session_id << "', '"<< instance._peer_id << "') signaling change - " << new_state);

    // callback
    if (instance._callbacks.on_signaling_state_change.exists()) {
        ros_webrtc::OnSignalingStateChange srv;
        instance._callbacks.on_signaling_state_change.call(srv);
    }

    // event
    instance._events.on_signaling_state_change.publish(ros_webrtc::SignalingState());
}

void PeerConnection::PeerConnectionObserver::OnAddStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> stream) {
    ROS_INFO_STREAM("pc ('" << instance._session_id << "', '"<< instance._peer_id << "') add stream - label: " << stream->label());

    // audio track sink
    webrtc::AudioTrackVector audio_tracks(stream->GetAudioTracks());
    for (auto i = audio_tracks.begin(); i != audio_tracks.end(); i++) {
        std::string topic = join_names({
            "session_" + instance._session_id,
            "peer_" + instance._peer_id,
            "audio_" + (*i)->id()
        });
        AudioSinkPtr audio_sink(new AudioSink(
            instance._nh,
            topic,
            instance._queue_sizes.audio,
            (*i).get()
        ));
        instance._audio_sinks.push_back(audio_sink);
    }

    // video track sink
    webrtc::VideoTrackVector video_tracks(stream->GetVideoTracks());
    for (auto i = video_tracks.begin(); i != video_tracks.end(); i++) {
        std::string topic = join_names({
            "session_" + instance._session_id,
            "peer_" + instance._peer_id,
            "video_" + (*i)->id()
        });
        VideoRendererPtr video_renderer(new VideoRenderer(
            instance._nh,
            topic,
            instance._queue_sizes.video,
            (*i).get()
        ));
        instance._video_renderers.push_back(video_renderer);
    }

    // callback
    if (instance._callbacks.on_add_stream.exists()) {
        ros_webrtc::OnAddStream srv;
        instance._callbacks.on_add_stream.call(srv);
    }

    // event
    instance._events.on_add_stream.publish(ros_webrtc::Stream());
}

void PeerConnection::PeerConnectionObserver::OnRemoveStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> stream) {
    ROS_INFO_STREAM("pc ('" << instance._session_id << "', '"<< instance._peer_id << "') remove stream - label: " << stream->label());
    instance._pc->RemoveStream(stream);

    // audio track sink
    webrtc::AudioTrackVector audio_tracks(stream->GetAudioTracks());
    for (webrtc::AudioTrackVector::iterator i = audio_tracks.begin();
         i != audio_tracks.end();
         i++) {
        for (AudioSinks::iterator j = instance._audio_sinks.begin();
             j != instance._audio_sinks.end();
             j++) {
            if ((*i)->id() == (*j)->audio_track()->id()) {
                instance._audio_sinks.erase(j);
                break;
            }
        }
    }

    // video track sink
    webrtc::VideoTrackVector video_tracks(stream->GetVideoTracks());
    for (webrtc::VideoTrackVector::iterator i = video_tracks.begin(); i != video_tracks.end(); i++) {
        for (VideoRenderers::iterator j = instance._video_renderers.begin();
             j != instance._video_renderers.end();
             j++) {
            if ((*i)->id() == (*j)->video_track()->id()) {
                instance._video_renderers.erase(j);
                break;
            }
        }
    }

    // callback
    if (instance._callbacks.on_remove_stream.exists()) {
        ros_webrtc::OnRemoveStream srv;
        instance._callbacks.on_remove_stream.call(srv);
    }

    // event
    instance._events.on_remove_stream.publish(ros_webrtc::Stream());
}

void PeerConnection::PeerConnectionObserver::OnDataChannel(rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) {
    ROS_INFO_STREAM(
        "pc ('" << instance._session_id << "', '"<< instance._peer_id << "') " <<
        "data channel - id: " << data_channel->id() << " label: " << data_channel->label()
    );
    MediaType media_type;
    if (!data_channel->protocol().empty()) {
        if (!MediaType::matches(data_channel->protocol())) {
            ROS_WARN_STREAM(
                "pc (session_id='" << instance._session_id << "', peer_id='"<< instance._peer_id << "') "
                "data channel w/ label='" << data_channel->label() << "'" <<
                "has non media-type protocol='" << data_channel->protocol() << "'.";
            );
        } else {
            media_type = MediaType(data_channel->protocol());
        }
    }

    DataChannelPtr dc(new DataChannel(
        instance._nh,
        instance.topic("data_" + data_channel->label()),
        data_channel,
        media_type,
        instance._queue_sizes.data
    ));
    instance._dcs[data_channel->label()] = dc;

    // callback
    if (instance._callbacks.on_data_channel.exists()) {
        ros_webrtc::OnDataChannel srv;
        srv.request.data_channel = *dc;
        instance._callbacks.on_data_channel.call(srv);
    }

    // event
    ros_webrtc::DataChannel msg = *dc;
    instance._events.on_data_channel.publish(msg);
}

void PeerConnection::PeerConnectionObserver::OnRenegotiationNeeded() {
    ROS_INFO_STREAM("pc ('" << instance._session_id << "', '"<< instance._peer_id << "') re-negotiation needed");

    // callback
    if (instance._callbacks.on_negotiation_needed.exists()) {
        ros_webrtc::OnNegotiationNeeded srv;
        instance._callbacks.on_negotiation_needed.call(srv);
    }

    // event
    instance._events.on_negotiation_needed.publish(std_msgs::Empty());
}

void PeerConnection::PeerConnectionObserver::OnIceConnectionChange(webrtc::PeerConnectionInterface::IceConnectionState new_state) {
    ROS_INFO_STREAM("pc ('" << instance._session_id << "', '"<< instance._peer_id << "') ice connection state - " << new_state);
    instance._ice_connection_changed_at = ros::Time::now().toSec();

    if (instance._observer != NULL) {
        instance._observer->on_connection_change(new_state);
    }

    // callback
    if (instance._callbacks.on_ice_connection_state_change.exists()) {
        ros_webrtc::OnIceConnectionStateChange srv;
        instance._callbacks.on_ice_connection_state_change.call(srv);
    }

    // event
    ros_webrtc::IceConnectionState msg;
    instance._events.on_ice_connection_state_change.publish(msg);
}

void PeerConnection::PeerConnectionObserver::OnIceGatheringChange(webrtc::PeerConnectionInterface::IceGatheringState new_state) {
    ROS_INFO_STREAM("pc ('" << instance._session_id << "', '"<< instance._peer_id << "') ice gathering state - " << new_state);

    // callback
    if (instance._callbacks.on_ice_connection_state_change.exists()) {
        ros_webrtc::OnIceConnectionStateChange srv;
        instance._callbacks.on_ice_connection_state_change.call(srv);
    }

    // event
    ros_webrtc::IceConnectionState msg;
    instance._events.on_ice_connection_state_change.publish(msg);
}

void PeerConnection::PeerConnectionObserver::OnIceCandidate(const webrtc::IceCandidateInterface* candidate) {
    ros_webrtc::IceCandidate msg;
    msg.sdp_mid = candidate->sdp_mid();
    msg.sdp_mline_index = candidate->sdp_mline_index();
    candidate->ToString(&msg.candidate);

    // callback
    if (instance._callbacks.on_ice_candidate.exists()) {
        ros_webrtc::OnIceCandidate srv;
        srv.request.candidate = msg;
        instance._callbacks.on_ice_candidate.call(srv);
    }

    // event
    instance._events.on_ice_candidate.publish(msg);
}

void PeerConnection::PeerConnectionObserver::OnIceCandidatesRemoved(const std::vector<cricket::Candidate>& candidates) {
    ROS_INFO_STREAM(
        "pc ('" << instance._session_id << "', '"<< instance._peer_id << "') " <<
        "ice candidates removed"
    );
}

void PeerConnection::PeerConnectionObserver::OnIceConnectionReceivingChange(bool receiving) {
    ROS_INFO_STREAM(
        "pc ('" << instance._session_id << "', '"<< instance._peer_id << "') " <<
        "ice connection receiving change - " <<
        receiving
    );
}

// PeerConnection::CreateSessionDescriptionObserver


PeerConnection::CreateSessionDescriptionObserver::CreateSessionDescriptionObserver(PeerConnection &instance_) : instance(instance_) {
}

PeerConnection::CreateSessionDescriptionObserver::~CreateSessionDescriptionObserver() {
}

void PeerConnection::CreateSessionDescriptionObserver::OnSuccess(webrtc::SessionDescriptionInterface* desc) {
    // We now own |desc| by the calling convention
    std::unique_ptr<webrtc::SessionDescriptionInterface> own(desc);
    ROS_INFO_STREAM("pc('" << instance._session_id << "', '"<< instance._peer_id << "') create session description succeeded");
    if (instance._local_desc != NULL) {
        ROS_INFO_STREAM("local sdp already set, skipping");
        return;
    }
    std::string sdp;
    desc->ToString(&sdp);
    // FIXME: capture and log errors
    instance._local_desc.reset(webrtc::CreateSessionDescription(desc->type(), sdp, NULL));
    if (instance._pc != NULL) {
        ROS_INFO_STREAM("setting local sdp, type - " << desc->type());
        instance._pc->SetLocalDescription(
            instance._ssdo,
            // FIXME: capture and log errors
            webrtc::CreateSessionDescription(desc->type(), sdp, NULL)
        );
    }
}

void PeerConnection::CreateSessionDescriptionObserver::OnFailure(const std::string& error) {
    ROS_INFO_STREAM("pc('" << instance._session_id << "', '"<< instance._peer_id << "') create session failed - " << error);
}

// PeerConnection::SetSessionDescriptionObserver

PeerConnection::SetSessionDescriptionObserver::SetSessionDescriptionObserver(PeerConnection &instance_) :
    instance(instance_) {
}

PeerConnection::SetSessionDescriptionObserver::~SetSessionDescriptionObserver() {
}

void PeerConnection::SetSessionDescriptionObserver::OnSuccess() {
    ROS_INFO_STREAM("pc('" << instance._session_id << "', '"<< instance._peer_id << "') set session description succeeded");
    if (instance._pc == NULL)
        return;
    if (instance.is_offerer()) {
        if (instance._pc->remote_description() == NULL) {
            ROS_INFO_STREAM("pc('" << instance._session_id << "', '"<< instance._peer_id << "') local sdp set succeeded");
            instance._on_local_description(instance._local_desc.get());
        } else {
            ROS_INFO_STREAM("remote sdp set succeeded");
            instance._drain_remote_ice_candidates();
        }
    }
    else {
        if (instance._pc->local_description() != NULL) {
            ROS_INFO_STREAM("pc('" << instance._session_id << "', '"<< instance._peer_id << "') local sdp set succeeded");
            instance._on_local_description(instance._local_desc.get());
            instance._drain_remote_ice_candidates();
        } else {
        }
    }
}

void PeerConnection::SetSessionDescriptionObserver::OnFailure(const std::string& error) {
    ROS_INFO_STREAM("pc('" << instance._session_id << "', '"<< instance._peer_id << "') set description failed - " << error);
}

// PeerConnection::Events

PeerConnection::Events::Events(PeerConnection &pc) :
    on_data_channel(pc._nh.advertise<ros_webrtc::DataChannel>("data_channel", pc._queue_sizes.event)),
    on_negotiation_needed(pc._nh.advertise<std_msgs::Empty>("negotiation_needed", pc._queue_sizes.event)),
    on_ice_candidate(pc._nh.advertise<ros_webrtc::IceCandidate>("ice_candidate", pc._queue_sizes.event)),
    on_ice_connection_state_change(pc._nh.advertise<ros_webrtc::IceConnectionState>("ice_connection_state_change", pc._queue_sizes.event)),
    on_signaling_state_change(pc._nh.advertise<ros_webrtc::SignalingState>("signaling_state_change", pc._queue_sizes.event)),
    on_add_stream(pc._nh.advertise<ros_webrtc::Stream>("add_stream", pc._queue_sizes.event)),
    on_remove_stream(pc._nh.advertise<ros_webrtc::Stream>("remove_stream", pc._queue_sizes.event)),
    on_set_session_description(pc._nh.advertise<ros_webrtc::SessionDescription>("set_session_description", pc._queue_sizes.event)),
    on_close(pc._nh.advertise<ros_webrtc::Close>("close", pc._queue_sizes.event)) {
}

void PeerConnection::Events::shutdown() {
    on_data_channel.shutdown();
    on_negotiation_needed.shutdown();
    on_ice_candidate.shutdown();
    on_ice_connection_state_change.shutdown();
    on_signaling_state_change.shutdown();
    on_set_session_description.shutdown();
    on_add_stream.shutdown();
    on_remove_stream.shutdown();
    on_close.shutdown();
}

// PeerConnection::Callbacks

PeerConnection::Callbacks::Callbacks(PeerConnection &pc) :
    on_data_channel(pc._nh.serviceClient<ros_webrtc::OnDataChannel>(pc.callback("on_data_channel"))),
    on_negotiation_needed(pc._nh.serviceClient<ros_webrtc::OnNegotiationNeeded>(pc.callback("on_negotiation_needed"))),
    on_ice_candidate(pc._nh.serviceClient<ros_webrtc::OnIceCandidate>(pc.callback("on_ice_candidate"))),
    on_ice_connection_state_change(pc._nh.serviceClient<ros_webrtc::OnIceConnectionStateChange>(pc.callback("on_ice_connection_state_change"))),
    on_signaling_state_change(pc._nh.serviceClient<ros_webrtc::OnSignalingStateChange>(pc.callback("on_signaling_state_change"))),
    on_add_stream(pc._nh.serviceClient<ros_webrtc::OnAddStream>(pc.callback("on_add_stream"))),
    on_remove_stream(pc._nh.serviceClient<ros_webrtc::OnRemoveStream>(pc.callback("on_remove_stream"))),
    on_set_session_description(pc._nh.serviceClient<ros_webrtc::OnSetSessionDescription>(pc.callback("on_set_session_description"))) {
}

void PeerConnection::Callbacks::shutdown() {
    on_data_channel.shutdown();
    on_negotiation_needed.shutdown();
    on_ice_candidate.shutdown();
    on_ice_connection_state_change.shutdown();
    on_signaling_state_change.shutdown();
    on_add_stream.shutdown();
    on_remove_stream.shutdown();
    on_set_session_description.shutdown();
}
