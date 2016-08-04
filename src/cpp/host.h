#ifndef ROS_WEBRTC_HOST_H_
#define ROS_WEBRTC_HOST_H_

#include <memory>

#include <ros/callback_queue_interface.h>
#include <ros_webrtc/AddIceCandidate.h>
#include <ros_webrtc/CreateDataChannel.h>
#include <ros_webrtc/CreateOffer.h>
#include <ros_webrtc/CreatePeerConnection.h>
#include <ros_webrtc/DeletePeerConnection.h>
#include <ros_webrtc/GetHost.h>
#include <ros_webrtc/GetPeerConnection.h>
#include <ros_webrtc/RotateVideoSource.h>
#include <ros_webrtc/SendData.h>
#include <ros_webrtc/SetRemoteDescription.h>
#include <webrtc/api/peerconnectioninterface.h>

#include "media_constraints.h"
#include "peer_connection.h"
#include "video_capture.h"

/**
 * \brief Description of a local video source.
 */
struct VideoSource {

    enum Type {
        NameType = 0,
        IdType,
        ROSType
    };

    VideoSource();

    VideoSource(
        Type type,
        const std::string& name,
        const std::string& label,
        const MediaConstraints& constraints,
        bool publish = false,
        int rotation = 0
    );

    Type type;

    std::string name;

    std::string label;

    MediaConstraints constraints;

    bool publish;

    int rotation;

    rtc::scoped_refptr<webrtc::VideoTrackSourceInterface> interface;

    rtc::scoped_refptr<webrtc::VideoCaptureModule> capture_module;

    VideoRendererPtr renderer;

};

/**
 * \brief Description of a local audio source.
 */
struct AudioSource {

    AudioSource();

    AudioSource(
        const std::string& label,
        const MediaConstraints& constraints,
        bool publish = false
    );

    std::string label;

    MediaConstraints constraints;

    bool publish;

    rtc::scoped_refptr<webrtc::AudioSourceInterface> interface;

    AudioSinkPtr sink;

};

/**
 * \brief Accesses local media and manages peer connections.
 */
class Host {

public:

    Host(
        ros::NodeHandle &nh,
        const std::vector<VideoSource>& video_srcs,
        const AudioSource& audio_src,
        const MediaConstraints& pc_constraints,
        double pc_bond_connect_timeout,
        double pc_bond_heartbeat_timeout,
        const std::vector<webrtc::PeerConnectionInterface::IceServer>& ice_servers,
        const QueueSizes& queue_sizes);

    Host(const Host& other);

    ~Host();

    bool open(bool media=false);

    bool is_open() const;

    void close();

    PeerConnectionPtr create_peer_connection(
        const std::string& node_name,
        const std::string& session_id,
        const std::string& peer_id,
        const MediaConstraints& sdp_constraints,
        const std::vector<std::string>& audio_sources,
        const std::vector<std::string>& video_sources
    );

    bool delete_peer_connection(
        const std::string& session_id,
        const std::string& peer_id
    );

    struct FlushStats {

        size_t reaped_data_messages;

        FlushStats& operator += (const PeerConnection::FlushStats & rhs);

    };

    FlushStats flush();

private:

    class Service {

    public:

        Service(Host& instance);

        void advertise();

        void shutdown();

        bool add_ice_candidate(ros::ServiceEvent<ros_webrtc::AddIceCandidate::Request, ros_webrtc::AddIceCandidate::Response>& event);

        bool create_data_channel(ros::ServiceEvent<ros_webrtc::CreateDataChannel::Request, ros_webrtc::CreateDataChannel::Response>& event);

        bool create_offer(ros::ServiceEvent<ros_webrtc::CreateOffer::Request, ros_webrtc::CreateOffer::Response>& event);

        bool create_peer_connection(ros::ServiceEvent<ros_webrtc::CreatePeerConnection::Request, ros_webrtc::CreatePeerConnection::Response>& event);

        bool delete_peer_connection(ros::ServiceEvent<ros_webrtc::DeletePeerConnection::Request, ros_webrtc::DeletePeerConnection::Response>& event);

        bool get_host(ros::ServiceEvent<ros_webrtc::GetHost::Request, ros_webrtc::GetHost::Response>& event);

        bool get_peer_connection(ros::ServiceEvent<ros_webrtc::GetPeerConnection::Request, ros_webrtc::GetPeerConnection::Response>& event);

        bool send_data(ros::ServiceEvent<ros_webrtc::SendData::Request, ros_webrtc::SendData::Response>& event);

        bool set_remote_description(ros::ServiceEvent<ros_webrtc::SetRemoteDescription::Request, ros_webrtc::SetRemoteDescription::Response>& event);

        bool rotate_video_source(ros::ServiceEvent<ros_webrtc::RotateVideoSource::Request, ros_webrtc::RotateVideoSource::Response>& event);

    private:

        Host &_instance;

        ros::V_ServiceServer _srvs;

    };

    class PeerConnectionObserver : public PeerConnection::Observer {

    public:

        PeerConnectionObserver(Host& instance, PeerConnectionPtr pc);

        ~PeerConnectionObserver();

    private:

        Host& _instance;

        PeerConnectionPtr _pc;

    // PeerConnection::Observer

    public:

        void on_connection_change(webrtc::PeerConnectionInterface::IceConnectionState state);

        void on_bond_broken();

    };

    struct PeerConnectionKey {

        bool operator < (const PeerConnectionKey& other) const {
            if (session_id != other.session_id)
                return session_id < other.session_id;
            return peer_id < other.peer_id;
        }

        std::string session_id;

        std::string peer_id;

    };

    class DeletePeerConnectionCallback : public ros::CallbackInterface {

    public:

        DeletePeerConnectionCallback(Host& instance, const PeerConnectionKey& key);

    private:

        Host& _instance;

        PeerConnectionKey _key;

    // ros::CallbackInterface

    public:

        virtual ros::CallbackInterface::CallResult call();

    };

    typedef std::map<PeerConnectionKey, PeerConnectionPtr> Sessions;

    bool _create_pc_factory();

    bool _open_media();

    bool _is_media_open() const;

    void _close_media();

    PeerConnectionPtr _find_peer_connection(const PeerConnectionKey& key);

    PeerConnectionConstPtr _find_peer_connection(const PeerConnectionKey& key) const;

    ros::NodeHandle _nh;

    AudioSource _audio_src;

    std::vector<VideoSource> _video_srcs;

    VideoCaptureModuleRegistryPtr _video_capture_modules;

    MediaConstraints _pc_constraints;

    double _pc_bond_connect_timeout;

    double _pc_bond_heartbeat_timeout;

    std::vector<webrtc::PeerConnectionInterface::IceServer> _ice_servers;

    QueueSizes _queue_sizes;

    std::unique_ptr<rtc::Thread> _network_thd;

    std::unique_ptr<rtc::Thread> _signaling_thd;

    std::unique_ptr<rtc::Thread> _worker_thd;

    rtc::scoped_refptr<webrtc::PeerConnectionFactoryInterface> _pc_factory;

    Sessions _pcs;

    Service _srv;

    bool _auto_close_media;

};

typedef boost::shared_ptr<Host> HostPtr;

/**
 * \brief Helper used to create a Host.
 */
struct HostFactory {

    Host operator()(ros::NodeHandle &nh);

    std::vector<VideoSource> video_srcs;

    AudioSource audio_src;

    double pc_bond_connect_timeout;

    double pc_bond_heartbeat_timeout;

    MediaConstraints pc_constraints;

    std::vector<webrtc::PeerConnectionInterface::IceServer> ice_servers;

    QueueSizes queue_sizes;
};

#endif  /* WEBRTC_HOST_H_ */
