#include "session.h"

#include <json/json.h>
#include <ros/ros.h>
#include <talk/app/webrtc/jsepicecandidate.h>

#include "device.h"
#include "util.h"

// Session

Session::Session(
    const std::string& peer_id,
    webrtc::MediaStreamInterface* local_stream,
    const MediaConstraints& sdp_constraints,
    ros::Publisher& dc_rpub,
    const std::vector<ros_webrtc::DataChannel>& data_channels,
    const std::map<std::string, std::string>& service_names
    ) :
    _peer_id(peer_id),
    _local_stream(local_stream),
    _sdp_constraints(sdp_constraints),
    _pco(*this),
    _csdo(new Session::CreateSessionDescriptionObserver(*this)),
    _ssdo(new Session::SetSessionDescriptionObserver(*this)),
    _is_offerer(false),
    _queue_remote_ice_candidates(true),
    _dc_rpub(dc_rpub),
    _service_names(service_names) {
    for (size_t i = 0; i < data_channels.size(); i++) {
        _dcs.push_back(DataChannel(data_channels[i]));
    }
}

const std::string& Session::peer_id() const {
    return _peer_id;
}

bool Session::begin(
        webrtc::PeerConnectionFactoryInterface* pc_factory,
        const webrtc::MediaConstraintsInterface* pc_constraints,
        const webrtc::PeerConnectionInterface::IceServers& ice_servers,
        Session::ObserverPtr observer
    ) {
    _observer = observer;
    if (!_open_peer_connection(pc_factory, pc_constraints, ice_servers)) {
        end();
        return false;
    }
    if (!_open_service_clients()) {
        end();
        return false;
    }
    return true;
}

void Session::end() {
    _observer.reset();
    _close_peer_connection();
    _close_service_clients();
}

bool Session::connect() {
    ros_webrtc::Connect srv;
    srv.request.peer_id = _peer_id;
    return _service_clis["connect"].call(srv);
}

bool Session::create_offer() {
    for (DataChannels::iterator i = _dcs.begin(); i != _dcs.end(); i++) {
        webrtc::DataChannelInit init;
        init.id = (*i).conf.id;
        init.ordered = (*i).conf.ordered;
        init.reliable = (*i).conf.reliable;
        (*i).provider = _pc->CreateDataChannel((*i).conf.label, &init);
        if ((*i).provider.get() == NULL) {
            ROS_ERROR_STREAM("peer " << _peer_id << " data channel '" << (*i).conf.label << "' create failed");
            return false;
        }
        boost::shared_ptr<DataObserver> data_observer;
        if ((*i).conf.chunk_size == 0) {
            data_observer.reset(new UnchunkedDataObserver(
                _dc_rpub, (*i).provider.get()
            ));
        } else {
            data_observer.reset(new ChunkedDataObserver(
                _dc_rpub, (*i).provider.get()
            ));
        }
        (*i).observers.push_back(data_observer);
    }
    _is_offerer = true;
    _pc->CreateOffer(_csdo, &_sdp_constraints);
    return true;
}

void Session::create_answer() {
    _is_offerer = false;
    _pc->CreateAnswer(_csdo, &_sdp_constraints);
}

bool Session::is_offerer() {
    return _is_offerer;
}

void Session::add_remote_ice_candidate(webrtc::IceCandidateInterface* candidate) {
    if (_queue_remote_ice_candidates) {
        std::string sdp;
        candidate->ToString(&sdp);
        IceCandidatePtr copy(webrtc::CreateIceCandidate(
            candidate->sdp_mid(),
            candidate->sdp_mline_index(),
            sdp
        ));
        _remote_ice_cadidates.push_back(copy);
    } else {
        _pc->AddIceCandidate(candidate);
    }
}

void Session::set_remote_session_description(webrtc::SessionDescriptionInterface* sdp) {
    _pc->SetRemoteDescription(_ssdo, sdp);
}

Session::DataChannel* Session::data_channel(const std::string& label) {
    for (DataChannels::iterator i = _dcs.begin(); i != _dcs.end(); i++) {
        if ((*i).conf.label == label) {
            return &(*i);
        }
    }
    return NULL;
}

const Session::DataChannel* Session::data_channel(const std::string& label) const {
    for (DataChannels::const_iterator i = _dcs.begin(); i != _dcs.end(); i++) {
        if ((*i).conf.label == label) {
            return &(*i);
        }
    }
    return NULL;
}

Session::Flush Session::flush() {
    Session::Flush flush;
    for (DataChannels::iterator i = _dcs.begin(); i != _dcs.end(); i++) {
        for (DataChannel::Observers::iterator j = (*i).observers.begin(); j != (*i).observers.end(); j++) {
            flush.reaped_data_messages += (*j)->reap();
        }
    }
    return flush;
}

bool Session::_open_peer_connection(
        webrtc::PeerConnectionFactoryInterface* pc_factory,
        const webrtc::MediaConstraintsInterface* pc_constraints,
        const webrtc::PeerConnectionInterface::IceServers& ice_servers
    ) {
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

void Session::_close_peer_connection() {
    if (_service_clis.find("disconnect") != _service_clis.end()) {
        ros_webrtc::Disconnect srv;
        srv.request.peer_id = _peer_id;
        if (!_service_clis["disconnect"].call(srv)) {
            ROS_ERROR_STREAM("peer " << _peer_id << " 'disconnect' call failed");
        }
    }
    while (!_audio_sinks.empty()) {
        _audio_sinks.pop_front();
    }
    while (!_video_renderers.empty()) {
        _video_renderers.pop_front();
    }
    for (DataChannels::iterator i = _dcs.begin(); i != _dcs.end(); i++) {
        while (!(*i).observers.empty()) {
            (*i).observers.pop_front();
        }
        (*i).provider = NULL;
    }
    if (_pc != NULL) {
        _pc->Close();
        _pc = NULL;
    }
}

bool Session::_open_service_clients() {
    if (_service_names.find("disconnect") == _service_names.end()) {
        return false;
    }
    ROS_INFO_STREAM("peer " << _peer_id << " 'disconnect' service '" <<  _service_names["disconnect"] << "'");
    _service_clis["disconnect"] = _nh.serviceClient<ros_webrtc::Disconnect>(_service_names["disconnect"]);

    if (_service_names.find("ice_candidate") == _service_names.end()) {
        return false;
    }
    ROS_INFO_STREAM("peer " << _peer_id << " 'ice_candidate' service '" <<  _service_names["ice_candidate"] << "'");
    _service_clis["ice_candidate"] = _nh.serviceClient<ros_webrtc::IceCandidate>(_service_names["ice_candidate"]);

    if (_service_names.find("sdp_offer_answer") == _service_names.end()) {
        return false;
    }
    ROS_INFO_STREAM("peer " << _peer_id << " 'sdp_offer_answer' service '" <<  _service_names["sdp_offer_answer"] << "'");
    _service_clis["sdp_offer_answer"] = _nh.serviceClient<ros_webrtc::SdpOfferAnswer>(_service_names["sdp_offer_answer"]);

    return true;
}

void Session::_close_service_clients() {
    for (ServiceClients::iterator i = _service_clis.begin(); i != _service_clis.end(); i++) {
        (*i).second.shutdown();
    }
}

void Session::_on_local_description(webrtc::SessionDescriptionInterface* desc) {
    ROS_INFO_STREAM("peer " << _peer_id << " local description");
    ros_webrtc::SdpOfferAnswer srv;
    srv.request.peer_id = _peer_id;
    desc->ToString(&srv.request.sdp);
    if (is_offerer())
        srv.request.type = "offer";
    else
        srv.request.type = "answer";
    if (!_service_clis["sdp_offer_answer"].call(srv)) {
        ROS_ERROR_STREAM("peer " << _peer_id << " 'sdp_offer_answer' call failed");
    }
}

void Session::_drain_remote_ice_candidates() {
    ROS_INFO_STREAM(
        "peer "
        << _peer_id
        << " adding "
        << _remote_ice_cadidates.size()
        << " queued remote ice candidates"
    );
    while (!_remote_ice_cadidates.empty()) {
        IceCandidatePtr ice_candidate = _remote_ice_cadidates.front();
        _remote_ice_cadidates.pop_front();
        _pc->AddIceCandidate(ice_candidate.get());
    }
    _queue_remote_ice_candidates = false;
}

// Session::DataChannel

Session::DataChannel::DataChannel(const ros_webrtc::DataChannel& conf_) :
    conf(conf_) {

}

void Session::DataChannel::send(webrtc::DataBuffer& data_buffer, bool transfer) {
    rtc::scoped_refptr<webrtc::DataChannelInterface> provider_ = provider;
    if (provider_.get() == NULL) {
        ROS_INFO(
            "data channel '%s' has no provider, dropping message (%zu)... ",
            conf.label.c_str(), data_buffer.size()
        );
        return;
    }
    if (conf.chunk_size == 0) {
        provider_->Send(data_buffer);
    } else {
        // TODO: rate limit?
        ChunkedDataTransfer xfer(generate_id(), data_buffer, conf.chunk_size);
        while (!xfer.is_complete()) {
            xfer.send(provider);
        }
    }
}

// Session::ChunkedDataTransfer

Session::ChunkedDataTransfer::ChunkedDataTransfer(
    const std::string& id,
    const webrtc::DataBuffer& data_buffer,
    size_t size
    ) :
    id(id),
    data(data_buffer.data),
    size(size),
    current(0),
    total(static_cast<size_t>(std::ceil((double)data.length() / (double)size))) {
}

bool Session::ChunkedDataTransfer::is_complete() const {
    return current == total;
}

size_t Session::ChunkedDataTransfer::send(webrtc::DataChannelInterface* provider) {
    size_t bytes = 0;

    Json::Value chunk;
    chunk["id"] = id;
    chunk["index"] = static_cast<Json::UInt>(current);
    chunk["total"] = static_cast<Json::UInt>(total);
    chunk["data"] = std::string(
        &data.data()[0] + current * size,
        std::min(size, data.length() - current * size)
    );
    std::string serialized = chunk.toStyledString();
    webrtc::DataBuffer data_buffer(
        rtc::Buffer(reinterpret_cast<void *>(&serialized[0]), serialized.size()),
        false  // NOTE: == utf-8
    );
    provider->Send(data_buffer);
    bytes += data_buffer.size();
    current++;

    return bytes;
}

// Session::PeerConnectionObserver

Session::PeerConnectionObserver::PeerConnectionObserver(Session& instance_) : instance(instance_) {
}

void Session::PeerConnectionObserver::OnSignalingChange(webrtc::PeerConnectionInterface::SignalingState new_state) {
    ROS_INFO_STREAM("peer " << instance._peer_id << " signaling change - " << new_state);
}

void Session::PeerConnectionObserver::OnStateChange(StateType state_changed) {
    ROS_INFO_STREAM("peer " << instance._peer_id << " state - " << state_changed);
}

void Session::PeerConnectionObserver::OnAddStream(webrtc::MediaStreamInterface* stream) {
    ROS_INFO_STREAM("peer " << instance._peer_id << " add stream - label: " << stream->label());

    // audio track sink
    webrtc::AudioTrackVector audio_tracks(stream->GetAudioTracks());
    for (webrtc::AudioTrackVector::iterator i = audio_tracks.begin();
         i != audio_tracks.end();
         i++) {
        AudioSinkPtr audio_sink(new AudioSink(
            instance._nh,
            topic_for("session_" + instance._peer_id, "audio_" + (*i)->id()),
            (*i).get()
        ));
        instance._audio_sinks.push_back(audio_sink);
    }

    // video track sink
    webrtc::VideoTrackVector video_tracks(stream->GetVideoTracks());
    for (webrtc::VideoTrackVector::iterator i = video_tracks.begin();
         i != video_tracks.end();
         i++) {
        VideoRendererPtr video_renderer(new VideoRenderer(
            instance._nh,
            topic_for("session_" + instance._peer_id, "video_" + (*i)->id()),
            (*i).get()
        ));
        instance._video_renderers.push_back(video_renderer);
    }

}

void Session::PeerConnectionObserver::OnRemoveStream(webrtc::MediaStreamInterface* stream) {
    ROS_INFO_STREAM("peer " << instance._peer_id << " remove stream - label: " << stream->label());
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
        for (VideoRenderers::iterator j = instance._video_renderers.begin(); j != instance._video_renderers.end(); j++) {
            if ((*i)->id() == (*j)->video_track()->id()) {
                instance._video_renderers.erase(j);
                break;
            }
        }
    }
}

void Session::PeerConnectionObserver::OnDataChannel(webrtc::DataChannelInterface* data_channel) {
    ROS_INFO_STREAM(
        "peer " << instance._peer_id << " data channel - "
        << "id: " << data_channel->id() << " "
        << "label: " << data_channel->label()
    );
    for (DataChannels::iterator i = instance._dcs.begin(); i != instance._dcs.end(); i++) {
        if ((*i).conf.label != data_channel->label()) {
            continue;
        }
        (*i).provider = data_channel;
        boost::shared_ptr<DataObserver> data_observer;
        if ((*i).conf.chunk_size == 0) {
            data_observer.reset(new UnchunkedDataObserver(
                instance._dc_rpub, data_channel
            ));
        } else {
            data_observer.reset(new ChunkedDataObserver(
                instance._dc_rpub, data_channel
            ));
        }
        (*i).observers.push_back(data_observer);
    }
}

void Session::PeerConnectionObserver::OnRenegotiationNeeded() {
    ROS_INFO_STREAM("peer " << instance._peer_id << " re-negotiation needed");
}

void Session::PeerConnectionObserver::OnIceConnectionChange(webrtc::PeerConnectionInterface::IceConnectionState new_state) {
    ROS_INFO_STREAM("peer " << instance._peer_id << " ice connection state - " << new_state);
    if (instance._observer != NULL)
        instance._observer->on_connection_change(new_state);
}

void Session::PeerConnectionObserver::OnIceGatheringChange(webrtc::PeerConnectionInterface::IceGatheringState new_state) {
    ROS_INFO_STREAM("peer " << instance._peer_id << " ice gathering state - " << new_state);
}

void Session::PeerConnectionObserver::OnIceCandidate(const webrtc::IceCandidateInterface* candidate) {
    ros_webrtc::IceCandidate srv;
    srv.request.peer_id = instance._peer_id;
    srv.request.sdp_mid = candidate->sdp_mid();
    srv.request.sdp_mline_index = candidate->sdp_mline_index();
    candidate->ToString(&srv.request.candidate);
    if (!instance._service_clis["ice_candidate"].call(srv)) {
        ROS_ERROR_STREAM("peer " << instance._peer_id << " 'sdp_offer_answer' call failed");
    }
}

void Session::PeerConnectionObserver::OnIceComplete() {
    ROS_INFO_STREAM("peer " << instance._peer_id << " ice complete");
}

// Session::CreateSessionDescriptionObserver


Session::CreateSessionDescriptionObserver::CreateSessionDescriptionObserver(Session &instance_) : instance(instance_) {
}

Session::CreateSessionDescriptionObserver::~CreateSessionDescriptionObserver() {
}

void Session::CreateSessionDescriptionObserver::OnSuccess(webrtc::SessionDescriptionInterface* desc) {
    ROS_INFO_STREAM("peer " << instance._peer_id << " create session description succeeded");
    if (instance._local_desc != NULL) {
        ROS_INFO_STREAM("local sdp already set, skipping");
        return;
    }
    std::string sdp;
    desc->ToString(&sdp);
    instance._local_desc.reset(webrtc::CreateSessionDescription(desc->type(), sdp));
    if (instance._pc != NULL) {
        ROS_INFO_STREAM("setting local sdp, type - " << desc->type());
        instance._pc->SetLocalDescription(instance._ssdo, webrtc::CreateSessionDescription(desc->type(), sdp));
    }
}

void Session::CreateSessionDescriptionObserver::OnFailure(const std::string& error) {
    ROS_INFO_STREAM("peer " << instance._peer_id << " create session failed - " << error);
}

// Session::SetSessionDescriptionObserver

Session::SetSessionDescriptionObserver::SetSessionDescriptionObserver(Session &instance_) :
    instance(instance_) {
}

Session::SetSessionDescriptionObserver::~SetSessionDescriptionObserver() {
}

void Session::SetSessionDescriptionObserver::OnSuccess() {
    ROS_INFO_STREAM("peer " << instance._peer_id << " set session succeeded");
    if (instance._pc == NULL)
        return;
    if (instance.is_offerer()) {
        if (instance._pc->remote_description() == NULL) {
            ROS_INFO_STREAM("peer " << instance._peer_id << " local sdp set succeeded");
            instance._on_local_description(instance._local_desc.get());
        } else {
            ROS_INFO_STREAM("remote sdp set succeeded");
            instance._drain_remote_ice_candidates();
        }
    }
    else {
        if (instance._pc->local_description() != NULL) {
            ROS_INFO_STREAM("peer " << instance._peer_id << " local sdp set succeeded");
            instance._on_local_description(instance._local_desc.get());
            instance._drain_remote_ice_candidates();
        } else {
            ROS_INFO_STREAM("peer << instance._peer_id << remote sdp set succeeded");
        }
    }
}

void Session::SetSessionDescriptionObserver::OnFailure(const std::string& error) {
    ROS_INFO_STREAM("peer " << instance._peer_id << " set session failed - " << error);
}
