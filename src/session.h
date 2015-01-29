#ifndef WEBRTC_SESSION_H_
#define WEBRTC_SESSION_H_

#include <list>

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <ros_webrtc/DataChannel.h>
#include <talk/app/webrtc/peerconnectioninterface.h>
#include <webrtc/base/scoped_ptr.h>
#include <webrtc/base/refcount.h>
#include <webrtc/base/scoped_ref_ptr.h>

#include "media_constraints.h"
#include "renderer.h"

class Device;

/**
 * @class Device
 * @brief Represents a connection between this Device and a peer.
 */
class Session {

public:

    /**
     * @brief Creates an initial session.
     * @param peer_id Unique string identifying the remote peer.
     * @param local_stream The local video and audio tracks for the device as a stream to share with the peer.
     * @param sdp_constraints Media constraints to apply for this session.
     * @param dc_rpub The ROS publisher to use to publish sending data channel messages.
     * @param dc_inits Data channel settings (e.g. label, reliability, ordering, etc).
     * @param service_names Names of ROS services to use for signaling (e.g. ice-candidates, sdp-offer-answers, etc).
     */
    Session(
        const std::string& peer_id,
        webrtc::MediaStreamInterface* local_stream,
        const MediaConstraints& sdp_constraints,
        ros::Publisher& dc_rpub,
        const std::vector<ros_webrtc::DataChannel>& dc_inits,
        const std::map<std::string, std::string>& service_names
    );

    /**
     * @brief Unique string identifying the remote peer.
     * @return Remote peer identifier.
     */
    const std::string& peer_id() const;

    /**
     * @class Observer
     * @brief Callback interface used to observe session life-cycle.
     */
    class Observer {

    public:

        virtual ~Observer() {}

        virtual void on_connection_change(webrtc::PeerConnectionInterface::IceConnectionState state) = 0;

    };

    typedef boost::shared_ptr<Observer> ObserverPtr;

    /**
     * @brief Initiate connection to remote peer for this session.
     * @param pc_factory Factory used to create a peer connection.
     * @param pc_constraints Constraints to apply to the connection.
     * @param ice_servers Collection of servers to use for ICE (connection negotiation).
     * @param observer Optional observer to call on various session life-cycle events (e.g. disconnect).
     * @return Whether initiation of peer connection succeeded.
     */
    bool begin(
        webrtc::PeerConnectionFactoryInterface* pc_factory,
        const webrtc::MediaConstraintsInterface* pc_constraints,
        const webrtc::PeerConnectionInterface::IceServers& ice_servers,
        ObserverPtr observer = ObserverPtr()
    );

    /**
     * @brief Teardown connection to remote peer, ending this session.
     */
    void end();

    bool connect();

    void create_offer();

    bool is_offerer();

    void create_answer();

    void add_remote_ice_candidate(webrtc::IceCandidateInterface* candidate);

    void set_remote_session_description(webrtc::SessionDescriptionInterface* sdp);

    webrtc::DataChannelInterface* data_channel(const std::string& label);

    const webrtc::DataChannelInterface* data_channel(const std::string& label) const;

private:

    bool _open_peer_connection(
        webrtc::PeerConnectionFactoryInterface* pc_factory,
        const webrtc::MediaConstraintsInterface* pc_constraints,
        const webrtc::PeerConnectionInterface::IceServers& ice_servers
    );

    void _close_peer_connection();

    bool _open_service_clients();

    void _close_service_clients();

    void _on_local_description(webrtc::SessionDescriptionInterface* desc);

    void _drain_remote_ice_candidates();

    class PeerConnectionObserver : public webrtc::PeerConnectionObserver {

    public:

        PeerConnectionObserver(Session &instance_);

        Session& instance;

    // webrtc::PeerConnectionObserver

    public:
        virtual void OnSignalingChange(webrtc::PeerConnectionInterface::SignalingState new_state);

        virtual void OnStateChange(StateType state_changed);

        virtual void OnAddStream(webrtc::MediaStreamInterface* stream);

        virtual void OnRemoveStream(webrtc::MediaStreamInterface* stream);

        virtual void OnDataChannel(webrtc::DataChannelInterface* data_channel);

        virtual void OnRenegotiationNeeded();

        virtual void OnIceConnectionChange(webrtc::PeerConnectionInterface::IceConnectionState new_state);

        virtual void OnIceGatheringChange(webrtc::PeerConnectionInterface::IceGatheringState new_state);

        virtual void OnIceCandidate(const webrtc::IceCandidateInterface* candidate);

        virtual void OnIceComplete();
    };

    class CreateSessionDescriptionObserver :
        public rtc::RefCountedObject<webrtc::CreateSessionDescriptionObserver> {

    public:

        CreateSessionDescriptionObserver(Session &instance_);

        virtual ~CreateSessionDescriptionObserver();

        Session& instance;

    // webrtc::CreateSessionDescriptionObserver

    public:

        virtual void OnSuccess(webrtc::SessionDescriptionInterface* desc);

        virtual void OnFailure(const std::string& error);

    };

    class SetSessionDescriptionObserver :
            public rtc::RefCountedObject<webrtc::SetSessionDescriptionObserver> {

    public:

        SetSessionDescriptionObserver(Session &instance_);

        virtual ~SetSessionDescriptionObserver();

        Session& instance;

    // webrtc::SetSessionDescriptionObserver

    public:

        virtual void OnSuccess();

        virtual void OnFailure(const std::string& error);

    };

    typedef boost::shared_ptr<webrtc::IceCandidateInterface> IceCandidatePtr;

    ros::NodeHandle _nh;

    std::string _peer_id;

    MediaConstraints _sdp_constraints;

    rtc::scoped_refptr<webrtc::MediaStreamInterface> _local_stream;

    rtc::scoped_refptr<webrtc::PeerConnectionInterface> _pc;

    ros::ServiceClient _ice_candidate_cli;

    ros::ServiceClient _sdp_offer_answer_cli;

    ros::ServiceClient _disconnect_cli;

    PeerConnectionObserver _pco;

    ObserverPtr _observer;

    rtc::scoped_refptr<CreateSessionDescriptionObserver> _csdo;

    rtc::scoped_refptr<SetSessionDescriptionObserver> _ssdo;

    bool _is_offerer;

    rtc::scoped_ptr<webrtc::SessionDescriptionInterface> _local_desc;

    bool _queue_remote_ice_candidates;

    std::list<IceCandidatePtr> _remote_ice_cadidates;

    ros::Publisher _dc_rpub;

    typedef std::vector<ros_webrtc::DataChannel> DataChannelInits;

    DataChannelInits _dc_inits;

    typedef std::vector<rtc::scoped_refptr<webrtc::DataChannelInterface> > DataChannels;

    DataChannels _dcs;

    std::map<std::string, std::string> _service_names;

    typedef std::map<std::string, ros::ServiceClient> ServiceClients;

    ServiceClients _service_clis;

    typedef std::list<AudioSinkPtr> AudioSinks;

    AudioSinks _audio_sinks;

    typedef std::list<VideoRendererPtr> VideoRenderers;

    VideoRenderers _video_renderers;

    std::list<DataObserverPtr> _data_observers;
};

typedef boost::shared_ptr<Session> SessionPtr;

typedef boost::shared_ptr<const Session> SessionConstPtr;


#endif /* WEBRTC_SESSION_H_ */
