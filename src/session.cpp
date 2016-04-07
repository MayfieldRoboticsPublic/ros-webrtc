#include "session.h"

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <json/json.h>
#include <ros/ros.h>
#include <ros_webrtc/SessionEvent.h>
#include <talk/app/webrtc/jsepicecandidate.h>

#include "convert.h"
#include "host.h"
#include "util.h"

// Session

Session::Session(
    const std::string& id,
    const std::string& peer_id,
    const std::vector<ros_webrtc::DataChannel>& dcs,
    const MediaConstraints& sdp_constraints,
    const std::map<std::string, std::string>& service_names,
    const QueueSizes& queue_sizes) :
    _id(id),
    _peer_id(peer_id),
    _sdp_constraints(sdp_constraints),
    _pco(*this),
    _csdo(new Session::CreateSessionDescriptionObserver(*this)),
    _ssdo(new Session::SetSessionDescriptionObserver(*this)),
    _is_offerer(false),
    _queue_remote_ice_candidates(true),
    _srv_cli(*this, service_names),
    _queue_sizes(queue_sizes),
    _bond(
        "bond",
        id + "_" + _peer_id,
        boost::function<void (void)>(boost::bind(&Session::_on_bond_broken, this)),
        boost::function<void (void)>(boost::bind(&Session::_on_bond_formed, this))
    ),
    _epub(_nh.advertise<ros_webrtc::SessionEvent>("session_event", queue_sizes.event)) {
    for (size_t i = 0; i < dcs.size(); i++) {
        _dcs.push_back(DataChannel(dcs[i]));
    }
}

const std::string& Session::id() const {
    return _id;
}

const std::string& Session::peer_id() const {
    return _peer_id;
}

bool Session::begin(
    webrtc::PeerConnectionFactoryInterface* pc_factory,
    const webrtc::MediaConstraintsInterface* pc_constraints,
    const webrtc::PeerConnectionInterface::IceServers& ice_servers,
    const std::vector<AudioSource> &audio_srcs,
    const std::vector<VideoSource> &video_srcs,
    Session::ObserverPtr observer) {
    _observer = observer;
    if (!_open_local_stream(pc_factory, audio_srcs, video_srcs)) {
        end();
        return false;
    }
    if (!_open_peer_connection(pc_factory, pc_constraints, ice_servers)) {
        end();
        return false;
    }
    _bond.start();
    _begin_event();
    return true;
}

void Session::end() {
    _end_event();
    _close_peer_connection();
    _close_local_stream();
    _observer.reset();
    _srv_cli.shutdown();
    _bond.breakBond();
}

webrtc::PeerConnectionInterface* Session::peer_connection() {
    return _pc.get();
}

bool Session::create_offer() {
    for (DataChannels::iterator i = _dcs.begin(); i != _dcs.end(); i++) {
        webrtc::DataChannelInit init;
        init.id = (*i).conf.id;
        init.protocol = (*i).conf.protocol;
        init.ordered = (*i).conf.ordered;
        init.reliable = (*i).conf.reliable;
        (*i).provider = _pc->CreateDataChannel((*i).conf.label, &init);
        if ((*i).provider.get() == NULL) {
            ROS_ERROR(
                "session '%s' data channel '%s' create failed",
                _id.c_str(), (*i).conf.label.c_str()
            );
            return false;
        }

        std::string send_topic = (*i).send_topic(*this);
        (*i).subscriber = _nh.subscribe<ros_webrtc::Data>(
            send_topic, _queue_sizes.data, &DataChannel::send, &(*i)
        );

        std::string recv_topic = (*i).recv_topic(*this);
        boost::shared_ptr<DataObserver> data_observer;
        if ((*i).is_chunked()) {
            data_observer.reset(new ChunkedDataObserver(
                _nh,
                recv_topic,
                _queue_sizes.data,
                (*i).provider.get()
            ));
        } else {
            data_observer.reset(new UnchunkedDataObserver(
                _nh,
                recv_topic,
                _queue_sizes.data,
                (*i).provider.get()
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

bool Session::is_offerer() const {
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

Session::operator ros_webrtc::Session () const {
    ros_webrtc::Session session;

    session.id = _id;

    session.peer_id = _peer_id;

    session.is_offerer = _is_offerer;

    session.sdp_constraints = _sdp_constraints;

    rtc::scoped_refptr<webrtc::PeerConnectionInterface> pc(_pc);
    if (pc.get() != NULL) {
        // local stream
        auto local_streams = pc->local_streams();
        for (size_t i = 0; i < local_streams->count(); i+= 1) {
            auto stream = local_streams->at(i);

            // audoio tracks
            auto a_tracks = stream->GetAudioTracks();
            for (auto iter = a_tracks.begin(); iter != a_tracks.end(); iter++) {
                auto & track = *iter;
                session.local_tracks.push_back(to_ros(track));
            }

            // video tracks
            auto v_tracks = stream->GetVideoTracks();
            for (auto iter = v_tracks.begin(); iter != v_tracks.end(); iter++) {
                auto & track = *iter;
                session.local_tracks.push_back(to_ros(track));
            }
        }

        // local stream
        auto remote_streams = pc->remote_streams();
        for (size_t i = 0; i < remote_streams->count(); i+= 1) {
            auto stream = remote_streams->at(i);

            // audoio tracks
            auto a_tracks = stream->GetAudioTracks();
            for (auto iter = a_tracks.begin(); iter != a_tracks.end(); iter++) {
                auto & track = *iter;
                session.remote_tracks.push_back(to_ros(track));
            }

            // video tracks
            auto v_tracks = stream->GetVideoTracks();
            for (auto iter = v_tracks.begin(); iter != v_tracks.end(); iter++) {
                auto & track = *iter;
                session.remote_tracks.push_back(to_ros(track));
            }
        }

        session.signaling_state = to_string(pc->signaling_state());

        session.ice_connection_state = to_string(pc->ice_connection_state());

        session.ice_gathering_state = to_string(pc->ice_gathering_state());
    }

    for (auto i = _dcs.begin(); i != _dcs.end(); i++) {
        session.data_channels.push_back(*i);
    }

    return session;
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

Session::FlushStats Session::flush() {
    Session::FlushStats flush;
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

void Session::_close_peer_connection() {
    ros_webrtc::EndSession srv;
    srv.request.session_id = _id;
    srv.request.peer_id = _peer_id;
    _srv_cli.end_session.call(srv);

    _close_local_stream();

    for (DataChannels::iterator i = _dcs.begin(); i != _dcs.end(); i++) {
        (*i).subscriber.shutdown();
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

bool Session::_open_local_stream(
        webrtc::PeerConnectionFactoryInterface* pc_factory,
        const std::vector<Session::AudioSource> &audio_srcs,
        const std::vector<Session::VideoSource> &video_srcs
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

void Session::_close_local_stream() {
    while (!_audio_sinks.empty()) {
        _audio_sinks.pop_front();
    }
    while (!_video_renderers.empty()) {
        _video_renderers.pop_front();
    }
    _local_stream = NULL;
}

void Session::_on_bond_formed() {
    ROS_INFO("session '%s' bond formed", _id.c_str());
}

void Session::_on_bond_broken() {
    ROS_INFO("session '%s' bond broken", _id.c_str());
}

void Session::_on_local_description(webrtc::SessionDescriptionInterface* desc) {
    ROS_INFO("session '%s' local description", _id.c_str());
    ros_webrtc::SetSessionDescription srv;
    srv.request.session_id = _id;
    srv.request.peer_id = _peer_id;
    desc->ToString(&srv.request.sdp);
    if (is_offerer())
        srv.request.type = "offer";
    else
        srv.request.type = "answer";
    if (!_srv_cli.set_session_description.call(srv)) {
        ROS_ERROR(
            "'set_session_description' call failed for session %s, peer %s",
            _id.c_str(), _peer_id.c_str()
        );
    }
}

void Session::_drain_remote_ice_candidates() {
    ROS_INFO("session '%s' adding %zu q'd remote ice candidates", _id.c_str(), _remote_ice_cadidates.size());
    while (!_remote_ice_cadidates.empty()) {
        IceCandidatePtr ice_candidate = _remote_ice_cadidates.front();
        _remote_ice_cadidates.pop_front();
        _pc->AddIceCandidate(ice_candidate.get());
    }
    _queue_remote_ice_candidates = false;
}

void Session::_begin_event() {
    ros_webrtc::SessionEvent e;
    e.id = _id;
    e.peer_id = _peer_id;
    e.type = ros_webrtc::SessionEvent::TYPE_BEGIN;
    _epub.publish(e);
}

void Session::_renegotiation_needed_event() {
    ros_webrtc::SessionEvent e;
    e.id = _id;
    e.peer_id = _peer_id;
    e.type = ros_webrtc::SessionEvent::TYPE_RENEGOTIATION_NEEDED;
    _epub.publish(e);
}

void Session::_signaling_state_change_event(webrtc::PeerConnectionInterface::SignalingState) {
    ros_webrtc::SessionEvent e;
    e.id = _id;
    e.peer_id = _peer_id;
    e.type = ros_webrtc::SessionEvent::TYPE_SIGNALING_STATE_CHANGE;
    _epub.publish(e);
}

void Session::_ice_state_change_event(webrtc::PeerConnectionInterface::IceState new_state) {
    ros_webrtc::SessionEvent e;
    e.id = _id;
    e.peer_id = _peer_id;
    e.type = ros_webrtc::SessionEvent::TYPE_ICE_STATE_CHANGE;
    _epub.publish(e);
}

void Session::_ice_connection_change_event(webrtc::PeerConnectionInterface::IceConnectionState new_state) {
    ros_webrtc::SessionEvent e;
    e.id = _id;
    e.peer_id = _peer_id;
    e.type = ros_webrtc::SessionEvent::TYPE_ICE_CONNECTION_CHANGE;
    _epub.publish(e);
}

void Session::_ice_gathering_change_event(webrtc::PeerConnectionInterface::IceGatheringState new_state) {
    ros_webrtc::SessionEvent e;
    e.id = _id;
    e.peer_id = _peer_id;
    e.type = ros_webrtc::SessionEvent::TYPE_ICE_GATHERING_CHANGE;
    _epub.publish(e);
}

void Session::_ice_complete_event() {
    ros_webrtc::SessionEvent e;
    e.id = _id;
    e.peer_id = _peer_id;
    e.type = ros_webrtc::SessionEvent::TYPE_ICE_COMPLETE;
    _epub.publish(e);
}

void Session::_add_stream_event() {
    ros_webrtc::SessionEvent e;
    e.id = _id;
    e.peer_id = _peer_id;
    e.type = ros_webrtc::SessionEvent::TYPE_ADD_STREAM;
    _epub.publish(e);
}

void Session::_remove_stream_event() {
    ros_webrtc::SessionEvent e;
    e.id = _id;
    e.peer_id = _peer_id;
    e.type = ros_webrtc::SessionEvent::TYPE_REMOVE_STREAM;
    _epub.publish(e);
}

void Session::_data_channel_event() {
    ros_webrtc::SessionEvent e;
    e.id = _id;
    e.peer_id = _peer_id;
    e.type = ros_webrtc::SessionEvent::TYPE_DATA_CHANNEL;
    _epub.publish(e);
}

void Session::_end_event() {
    ros_webrtc::SessionEvent e;
    e.id = _id;
    e.peer_id = _peer_id;
    e.type = ros_webrtc::SessionEvent::TYPE_END;
    _epub.publish(e);
}

// Session::VideoSource

Session::VideoSource::VideoSource(
    const std::string& label,
    webrtc::VideoSourceInterface *interface,
    bool publish) :
        label(label),
        interface(interface),
        publish(publish) {
}

// Session::AudioSource

Session::AudioSource::AudioSource(
    const std::string& label,
    webrtc::AudioSourceInterface *interface,
    bool publish) :
        label(label),
        interface(interface),
        publish(publish){
}

// Session::DataChannel

Session::DataChannel::DataChannel(const ros_webrtc::DataChannel& conf) :
    conf(conf),
    protocol(MediaType::parse(conf.protocol)) {
}

bool Session::DataChannel::is_chunked() const {
    return protocol.sub_type == "mayfield.msg.v1" && chunk_size() != 0;
}

size_t Session::DataChannel::chunk_size() const {
    auto i = protocol.params.find("chunksize");
    return i == protocol.params.end() ? 0 : std::atoi((*i).second.c_str());
}

std::string Session::DataChannel::send_topic(const Session& session) const {
    std::string topic;
    if (conf.broadcast) {
        topic = topic_for({
            "data_" + conf.label,
            "send"
        });
    } else {
        topic = topic_for({
            "session_" + session.id(),
            "peer_" + session.peer_id(),
            "data_" + conf.label,
            "send"
        });
    }
    return topic;
}

std::string Session::DataChannel::recv_topic(const Session& session) const {
    std::string topic;
    if (conf.broadcast) {
        topic = topic_for({
            "data_" + conf.label,
            "recv"
        });
    } else {
        topic = topic_for({
            "session_" + session.id(),
            "peer_" + session.peer_id(),
            "data_" + conf.label,
            "recv"
        });
    }
    return topic;
}

Session::DataChannel::operator ros_webrtc::DataChannel () const {
    ros_webrtc::DataChannel dst;
    dst.label = conf.label;
    dst.id = conf.id;
    dst.reliable = conf.reliable;
    dst.ordered = conf.ordered;
    dst.protocol = conf.protocol;
    dst.broadcast = conf.broadcast;
    dst.state = (provider != NULL ? to_string(provider->state()) : "closed");
    return dst;
}

void Session::DataChannel::send(const ros_webrtc::DataConstPtr& msg) {
    std::cerr.flush();
    webrtc::DataBuffer data_buffer(
        rtc::Buffer(&msg->buffer[0], msg->buffer.size()),
        msg->encoding == "binary"
    );
    send(data_buffer);
}

void Session::DataChannel::send(webrtc::DataBuffer& data_buffer) {
    rtc::scoped_refptr<webrtc::DataChannelInterface> provider_ = provider;
    if (provider_.get() == NULL) {
        ROS_INFO(
            "data channel '%s' has no provider, dropping message (%zu)... ",
            conf.label.c_str(), data_buffer.size()
        );
        return;
    }
    if (!is_chunked()) {
        provider_->Send(data_buffer);
    } else {
        ChunkedDataTransfer xfer(
            generate_id(), data_buffer, chunk_size() == 0 ? 128 : chunk_size()
        );
        while (!xfer.is_complete()) {
            // TODO: rate limit?
            xfer(provider);
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

size_t Session::ChunkedDataTransfer::operator()(webrtc::DataChannelInterface* provider) {
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
    ROS_INFO_STREAM("session " << instance._id << " signaling change - " << new_state);
    instance._signaling_state_change_event(new_state);
}

void Session::PeerConnectionObserver::OnStateChange(StateType state_changed) {
    ROS_INFO_STREAM("session " << instance._id << " state - " << state_changed);
    switch (state_changed) {
        case StateType::kIceState:
            instance._ice_state_change_event(instance._pc->ice_state());
            break;
        case StateType::kSignalingState:
            instance._signaling_state_change_event(instance._pc->signaling_state());
        break;
    }
}

void Session::PeerConnectionObserver::OnAddStream(webrtc::MediaStreamInterface* stream) {
    ROS_INFO_STREAM("session " << instance._id << " add stream - label: " << stream->label());
    instance._add_stream_event();

    // audio track sink
    webrtc::AudioTrackVector audio_tracks(stream->GetAudioTracks());
    for (auto i = audio_tracks.begin(); i != audio_tracks.end(); i++) {
        std::string topic = topic_for({
            "session_" + instance._id,
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
        std::string topic = topic_for({
            "session_" + instance._id,
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
}

void Session::PeerConnectionObserver::OnRemoveStream(webrtc::MediaStreamInterface* stream) {
    ROS_INFO_STREAM("session " << instance._id << " remove stream - label: " << stream->label());
    instance._remove_stream_event();
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
}

void Session::PeerConnectionObserver::OnDataChannel(webrtc::DataChannelInterface* data_channel) {
    ROS_INFO(
        "session %s data channel - id: %d label: %s",
        instance._id.c_str(), data_channel->id(), data_channel->label().c_str()
    );
    instance._data_channel_event();
    for (DataChannels::iterator i = instance._dcs.begin(); i != instance._dcs.end(); i++) {
        if ((*i).conf.label != data_channel->label()) {
            continue;
        }
        (*i).provider = data_channel;

        std::string send_topic = (*i).send_topic(instance);
        (*i).subscriber = instance._nh.subscribe<ros_webrtc::Data>(
            send_topic, instance._queue_sizes.data, &DataChannel::send, &(*i)
        );

        std::string recv_topic = (*i).recv_topic(instance);
        boost::shared_ptr<DataObserver> data_observer;
        if ((*i).is_chunked()) {
            data_observer.reset(new ChunkedDataObserver(
                instance._nh,
                recv_topic,
                instance._queue_sizes.data,
                data_channel
            ));
        } else {
            data_observer.reset(new UnchunkedDataObserver(
                instance._nh,
                recv_topic,
                instance._queue_sizes.data,
                data_channel
            ));
        }
        (*i).observers.push_back(data_observer);
    }
}

void Session::PeerConnectionObserver::OnRenegotiationNeeded() {
    ROS_INFO_STREAM("session " << instance._id << " re-negotiation needed");
    instance._renegotiation_needed_event();
}

void Session::PeerConnectionObserver::OnIceConnectionChange(webrtc::PeerConnectionInterface::IceConnectionState new_state) {
    ROS_INFO_STREAM("session " << instance._id << " ice connection state - " << new_state);
    instance._ice_connection_change_event(new_state);
    if (instance._observer != NULL)
        instance._observer->on_connection_change(new_state);
}

void Session::PeerConnectionObserver::OnIceGatheringChange(webrtc::PeerConnectionInterface::IceGatheringState new_state) {
    ROS_INFO_STREAM("session " << instance._id << " ice gathering state - " << new_state);
    instance._ice_gathering_change_event(new_state);
}

void Session::PeerConnectionObserver::OnIceCandidate(const webrtc::IceCandidateInterface* candidate) {
    ros_webrtc::AddSessionIceCandidate srv;
    srv.request.session_id = instance._id;
    srv.request.peer_id = instance._peer_id;
    srv.request.sdp_mid = candidate->sdp_mid();
    srv.request.sdp_mline_index = candidate->sdp_mline_index();
    candidate->ToString(&srv.request.candidate);
    if (!instance._srv_cli.add_session_ice_candidate.call(srv)) {
        ROS_ERROR_STREAM("session " << instance._id << " 'ice_candidate' call failed");
    }
}

void Session::PeerConnectionObserver::OnIceComplete() {
    ROS_INFO_STREAM("session " << instance._id << " ice complete");
    instance._ice_complete_event();
}

// Session::CreateSessionDescriptionObserver


Session::CreateSessionDescriptionObserver::CreateSessionDescriptionObserver(Session &instance_) : instance(instance_) {
}

Session::CreateSessionDescriptionObserver::~CreateSessionDescriptionObserver() {
}

void Session::CreateSessionDescriptionObserver::OnSuccess(webrtc::SessionDescriptionInterface* desc) {
    ROS_INFO_STREAM("session " << instance._id << " create session description succeeded");
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
    ROS_INFO_STREAM("session " << instance._id << " create session failed - " << error);
}

// Session::SetSessionDescriptionObserver

Session::SetSessionDescriptionObserver::SetSessionDescriptionObserver(Session &instance_) :
    instance(instance_) {
}

Session::SetSessionDescriptionObserver::~SetSessionDescriptionObserver() {
}

void Session::SetSessionDescriptionObserver::OnSuccess() {
    ROS_INFO_STREAM("session " << instance._id << " set session succeeded");
    if (instance._pc == NULL)
        return;
    if (instance.is_offerer()) {
        if (instance._pc->remote_description() == NULL) {
            ROS_INFO_STREAM("session " << instance._id << " local sdp set succeeded");
            instance._on_local_description(instance._local_desc.get());
        } else {
            ROS_INFO_STREAM("remote sdp set succeeded");
            instance._drain_remote_ice_candidates();
        }
    }
    else {
        if (instance._pc->local_description() != NULL) {
            ROS_INFO_STREAM("session " << instance._id << " local sdp set succeeded");
            instance._on_local_description(instance._local_desc.get());
            instance._drain_remote_ice_candidates();
        } else {
        }
    }
}

void Session::SetSessionDescriptionObserver::OnFailure(const std::string& error) {
    ROS_INFO_STREAM("session " << instance._id << " set description failed - " << error);
}

// Session::ServiceClient

Session::ServiceClient::ServiceClient(Session &instance, const std::map<std::string, std::string>& names) :
    _instance(instance),
    end_session(instance._nh.serviceClient<ros_webrtc::EndSession>(names.find("end_session")->second)),
    add_session_ice_candidate(instance._nh.serviceClient<ros_webrtc::AddSessionIceCandidate>(names.find("add_session_ice_candidate")->second)),
    set_session_description(instance._nh.serviceClient<ros_webrtc::SetSessionDescription>(names.find("set_session_description")->second)) {
}

void Session::ServiceClient::shutdown() {
    end_session.shutdown();
    add_session_ice_candidate.shutdown();
    set_session_description.shutdown();
}
