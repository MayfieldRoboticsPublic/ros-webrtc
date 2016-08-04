#ifndef ROS_WEBRTC_PEER_CONNECTION_H_
#define ROS_WEBRTC_PEER_CONNECTION_H_

#include <list>
#include <memory>

#include <boost/shared_ptr.hpp>
#include <bondcpp/bond.h>
#include <ros/ros.h>
#include <ros_webrtc/DataChannel.h>
#include <ros_webrtc/Data.h>
#include <ros_webrtc/PeerConnection.h>
#include <webrtc/api/mediaconstraintsinterface.h>
#include <webrtc/api/peerconnectioninterface.h>
#include <webrtc/media/base/videosourceinterface.h>
#include <webrtc/base/refcount.h>
#include <webrtc/base/scoped_ref_ptr.h>

#include "data_channel.h"
#include "media_constraints.h"
#include "renderer.h"

class Device;

/**
 * \brief Categorized sizes of publisher/subscriber queues.
 */
struct QueueSizes {

    QueueSizes(uint32_t size = 0);

    QueueSizes(uint32_t video, uint32_t audio, uint32_t data, uint32_t event);

    uint32_t video;
    uint32_t audio;
    uint32_t data;
    uint32_t event;

};

/**
 * \brief Represents a peer connection.
 */
class PeerConnection {

public:

    /**
     * \brief Creates an initial session.
     * \param node_name Name of node that created this peer connection.
     * \param session_id String identifying the session.
     * \param peer_id String identifying the remote peer.
     * \param sdp_constraints Media constraints to apply for this peer connection.
     * \param default_queue_size Default size of publisher and subscriber queues.
     * \param connect_timeout Bond connect timeout in seconds or 0 for no bonding.
     * \param heartbeat_timeout Bond heartbeat timeout in seconds or 0 for no bonding.
     */
    PeerConnection(
        const std::string& node_name,
        const std::string& session_id,
        const std::string& peer_id,
        const MediaConstraints& sdp_constraints,
        const QueueSizes& queue_sizes,
        double connect_timeout=10.0,
        double heartbeat_timeout=4.0);

    /**
     * \brief String identifying the session.
     * \return Session identifier.
     */
    const std::string& session_id() const;

    /**
     * \brief String identifying the remote peer for this session.
     * \return Peer identifier.
     */
    const std::string& peer_id() const;

    /**
     * \brief Prefix ROS topic name for topics associated with PeerConnection.
     * \return Prefixed ROS topic name.
     */
    std::string topic(const std::string &name) const;

    /**
     * \brief Prefix ROS callback service name for topics associated with PeerConnection.
     * \return Prefixed ROS callback service name.
     */
    std::string callback(const std::string &name) const;

    /**
     * \brief Video source for a PeerConnection to be added to local stream as a track.
     */
    struct VideoSource {

        VideoSource(
            const std::string& label,
            webrtc::VideoTrackSourceInterface *interface,
            bool publish = false
        );

        std::string label;

        bool publish;

        rtc::scoped_refptr<webrtc::VideoTrackSourceInterface> interface;

    };

    /**
     * \brief Audio source for a PeerConnection to be added to local stream as a track.
     */
    struct AudioSource {

        AudioSource(
            const std::string& label,
            webrtc::AudioSourceInterface *interface,
            bool publish = false
        );

        std::string label;

        rtc::scoped_refptr<webrtc::AudioSourceInterface> interface;

        bool publish;

    };

    /**
     * \brief Callback interface used to observe PeerConnection life-cycle.
     */
    class Observer {

    public:

        virtual ~Observer() {}

        virtual void on_connection_change(webrtc::PeerConnectionInterface::IceConnectionState state) = 0;

        virtual void on_bond_broken() = 0;

    };

    typedef boost::shared_ptr<Observer> ObserverPtr;

    /**
     * \brief Initiate connection to remote peer for this session.
     * \param pc_factory Factory used to create a peer connection.
     * \param pc_constraints Constraints to apply to the connection.
     * \param ice_servers Collection of servers to use for ICE (connection negotiation).
     * \param audio_srcs Local audio sources to share with peer as a stream tracks.
     * \param video_srcs Local video sources to share with peer as a stream tracks.
     * \param observer Optional observer to call on various PeerConnection life-cycle events (e.g. disconnect).
     * \return Whether initiation of peer connection succeeded.
     */
    bool begin(
        webrtc::PeerConnectionFactoryInterface* pc_factory,
        const webrtc::MediaConstraintsInterface* pc_constraints,
        const webrtc::PeerConnectionInterface::IceServers& ice_servers,
        const std::vector<AudioSource> &audio_srcs,
        const std::vector<VideoSource> &video_srcs,
        ObserverPtr observer = ObserverPtr());

    /**
     * \brief Teardown connection to remote peer, ending this session.
     */
    void end();

    webrtc::PeerConnectionInterface* peer_connection();

    bool create_data_channel(
        std::string label,
        std::string protocol=std::string(),
        bool reliable=false,
        bool ordered=false,
        int id=-1);

    DataChannelPtr data_channel(const std::string& label);

    DataChannelConstPtr data_channel(const std::string& label) const;

    void add_ice_candidate(webrtc::IceCandidateInterface* candidate);

    void set_remote_session_description(webrtc::SessionDescriptionInterface* sdp);

    bool create_offer();

    bool is_offerer() const;

    void create_answer();

    operator ros_webrtc::PeerConnection () const;

    struct FlushStats {

        size_t reaped_data_messages;

    };

    FlushStats flush();

private:

    class PeerConnectionObserver : public webrtc::PeerConnectionObserver {

    public:

        PeerConnectionObserver(PeerConnection &instance_);

        PeerConnection& instance;

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

        CreateSessionDescriptionObserver(PeerConnection &instance_);

        virtual ~CreateSessionDescriptionObserver();

        PeerConnection& instance;

    // webrtc::CreateSessionDescriptionObserver

    public:

        virtual void OnSuccess(webrtc::SessionDescriptionInterface* desc);

        virtual void OnFailure(const std::string& error);

    };

    class SetSessionDescriptionObserver :
            public rtc::RefCountedObject<webrtc::SetSessionDescriptionObserver> {

    public:

        SetSessionDescriptionObserver(PeerConnection &instance_);

        virtual ~SetSessionDescriptionObserver();

        PeerConnection& instance;

    // webrtc::SetSessionDescriptionObserver

    public:

        virtual void OnSuccess();

        virtual void OnFailure(const std::string& error);

    };

    typedef boost::shared_ptr<webrtc::IceCandidateInterface> IceCandidatePtr;

    class Events {

    public:

        Events(PeerConnection &pc);

        void shutdown();

        ros::Publisher on_data_channel;
        ros::Publisher on_negotiation_needed;
        ros::Publisher on_ice_candidate;
        ros::Publisher on_ice_connection_state_change;
        ros::Publisher on_signaling_state_change;
        ros::Publisher on_add_stream;
        ros::Publisher on_remove_stream;

        ros::Publisher on_set_session_description;
        ros::Publisher on_close;

    };

    class Callbacks {

    public:

        Callbacks(PeerConnection &pc);

        void shutdown();

        ros::ServiceClient on_data_channel;
        ros::ServiceClient on_negotiation_needed;
        ros::ServiceClient on_ice_candidate;
        ros::ServiceClient on_ice_connection_state_change;
        ros::ServiceClient on_signaling_state_change;
        ros::ServiceClient on_add_stream;
        ros::ServiceClient on_remove_stream;

        ros::ServiceClient on_set_session_description;

    };

    bool _open_peer_connection(
        webrtc::PeerConnectionFactoryInterface* pc_factory,
        const webrtc::MediaConstraintsInterface* pc_constraints,
        const webrtc::PeerConnectionInterface::IceServers& ice_servers);

    void _close_peer_connection();

    bool _open_local_stream(
        webrtc::PeerConnectionFactoryInterface* pc_factory,
        const std::vector<AudioSource> &audio_srcs,
        const std::vector<VideoSource> &video_srcs);

    void _close_local_stream();

    void _on_bond_formed();

    void _on_bond_broken();

    void _on_local_description(webrtc::SessionDescriptionInterface* desc);

    void _drain_remote_ice_candidates();

    void _begin_event();

    void _renegotiation_needed_event();

    void _signaling_state_change_event(webrtc::PeerConnectionInterface::SignalingState new_state);

    void _ice_state_change_event(webrtc::PeerConnectionInterface::IceState new_state);

    void _ice_gathering_change_event(webrtc::PeerConnectionInterface::IceGatheringState new_state);

    void _ice_connection_change_event(webrtc::PeerConnectionInterface::IceConnectionState new_state);

    void _ice_complete_event();

    void _add_stream_event();

    void _remove_stream_event();

    void _data_channel_event();

    void _end_event();

    ros::NodeHandle _nh;

    ros::Subscriber _s;

    std::string _nn;

    std::string _session_id;

    std::string _peer_id;

    QueueSizes _queue_sizes;

    bond::Bond _bond;

    MediaConstraints _sdp_constraints;

    std::vector<AudioSinkPtr> audio_sinks;

    std::vector<VideoRendererPtr> video_renderers;

    rtc::scoped_refptr<webrtc::MediaStreamInterface> _local_stream;

    rtc::scoped_refptr<webrtc::PeerConnectionInterface> _pc;

    PeerConnectionObserver _pco;

    ObserverPtr _observer;

    rtc::scoped_refptr<CreateSessionDescriptionObserver> _csdo;

    rtc::scoped_refptr<SetSessionDescriptionObserver> _ssdo;

    bool _is_offerer;

    std::unique_ptr<webrtc::SessionDescriptionInterface> _local_desc;

    bool _queue_remote_ice_candidates;

    std::list<IceCandidatePtr> _remote_ice_cadidates;

    std::map<std::string, DataChannelPtr> _dcs;

    typedef std::list<AudioSinkPtr> AudioSinks;

    AudioSinks _audio_sinks;

    typedef std::list<VideoRendererPtr> VideoRenderers;

    VideoRenderers _video_renderers;

    Events _events;

    Callbacks _callbacks;
};

typedef boost::shared_ptr<PeerConnection> PeerConnectionPtr;

typedef boost::shared_ptr<const PeerConnection> PeerConnectionConstPtr;


#endif /* ROS_WEBRTC_PEER_CONNECTION_H_ */
