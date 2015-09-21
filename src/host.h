#ifndef ROS_WEBRTC_HOST_H_
#define ROS_WEBRTC_HOST_H_

#include <ros/callback_queue_interface.h>
#include <ros_webrtc/AddSessionIceCandidate.h>
#include <ros_webrtc/BeginSession.h>
#include <ros_webrtc/ConnectSession.h>
#include <ros_webrtc/EndSession.h>
#include <ros_webrtc/GetHost.h>
#include <ros_webrtc/GetSession.h>
#include <ros_webrtc/SendSessionData.h>
#include <ros_webrtc/SetSessionDescription.h>
#include <talk/app/webrtc/peerconnectioninterface.h>

#include "media_constraints.h"
#include "session.h"

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
        bool publish = false
    );

    Type type;

    std::string name;

    std::string label;

    MediaConstraints constraints;

    bool publish;

    rtc::scoped_refptr<webrtc::VideoSourceInterface> interface;

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

};

/**
 * \brief Accesses local media and manages peer streaming sessions.
 */
class Host {

public:

    Host(
        ros::NodeHandle &nh,
        const std::vector<VideoSource>& video_srcs,
        const AudioSource& audio_src,
        const MediaConstraints& session_constraints,
        const std::vector<webrtc::PeerConnectionInterface::IceServer>& ice_servers,
        const QueueSizes& queue_sizes
    );

    Host(const Host& other);

    ~Host();

    bool open();

    bool is_open() const;

    void close();

    SessionPtr begin_session(
        const std::string& id,
        const std::string& peer_id,
        const MediaConstraints& sdp_constraints,
        const std::vector<std::string>& audio_sources,
        const std::vector<std::string>& video_sources,
        const std::vector<ros_webrtc::DataChannel>& data_channels,
        const std::map<std::string, std::string>& service_names
    );

    bool end_session(
        const std::string& id,
        const std::string& peer_id
    );

    struct FlushStats {

        size_t reaped_data_messages;

        FlushStats& operator += (const Session::FlushStats & rhs);

    };

    FlushStats flush();

private:

    class Service {

    public:

        Service(Host& instance);

        void advertise();

        void shutdown();

        bool begin_session(ros::ServiceEvent<ros_webrtc::BeginSession::Request, ros_webrtc::BeginSession::Response>& event);

        bool end_session(ros::ServiceEvent<ros_webrtc::EndSession::Request, ros_webrtc::EndSession::Response>& event);

        bool connect_session(ros::ServiceEvent<ros_webrtc::ConnectSession::Request, ros_webrtc::ConnectSession::Response>& event);

        bool add_session_ice_candidate(ros::ServiceEvent<ros_webrtc::AddSessionIceCandidate::Request, ros_webrtc::AddSessionIceCandidate::Response>& event);

        bool set_session_description(ros::ServiceEvent<ros_webrtc::SetSessionDescription::Request, ros_webrtc::SetSessionDescription::Response>& event);

        bool get_session(ros::ServiceEvent<ros_webrtc::GetSession::Request, ros_webrtc::GetSession::Response>& event);

        bool get_host(ros::ServiceEvent<ros_webrtc::GetHost::Request, ros_webrtc::GetHost::Response>& event);

    private:

        Host &_instance;

        ros::V_ServiceServer _srvs;

    };

    class SessionObserver : public Session::Observer {

    public:

        SessionObserver(Host& instance, SessionPtr session);

        ~SessionObserver();

    private:

        void _async_end_session(const ros::TimerEvent& event);

        Host& _instance;

        SessionPtr _session;

    // Session::Observer

    public:

        void on_connection_change(webrtc::PeerConnectionInterface::IceConnectionState state);

    };

    struct SessionKey {

        bool operator < (const SessionKey& other) const {
            if (id != other.id)
                return id < other.id;
            return peer_id < other.peer_id;
        }

        std::string id;

        std::string peer_id;

    };

    class EndSessionCallback : public ros::CallbackInterface {

    public:

        EndSessionCallback(Host& instance, const SessionKey& key);

    private:

        Host& _instance;

        SessionKey _key;

    // ros::CallbackInterface

    public:

        virtual ros::CallbackInterface::CallResult call();

    };

    typedef std::map<SessionKey, SessionPtr> Sessions;

    bool _create_pc_factory();

    bool _open_local_sources();

    void _close_local_sources();

    SessionPtr _find_session(const SessionKey& key);

    SessionConstPtr _find_session(const SessionKey& key) const;

    ros::NodeHandle _nh;

    AudioSource _audio_src;

    std::vector<VideoSource> _video_srcs;

    MediaConstraints _session_constraints;

    std::vector<webrtc::PeerConnectionInterface::IceServer> _ice_servers;

    QueueSizes _queue_sizes;

    rtc::scoped_ptr<rtc::Thread> _signaling_thd;

    rtc::scoped_ptr<rtc::Thread> _worker_thd;

    rtc::scoped_refptr<webrtc::PeerConnectionFactoryInterface> _pc_factory;

    Sessions _sessions;

    Service _srv;

};

typedef boost::shared_ptr<Host> HostPtr;

/**
 * \brief Helper used to create a Host.
 */
struct HostFactory {

    Host operator()(ros::NodeHandle &nh);

    std::vector<VideoSource> video_srcs;

    AudioSource audio_src;

    MediaConstraints session_constraints;

    std::vector<webrtc::PeerConnectionInterface::IceServer> ice_servers;

    QueueSizes queue_sizes;
};

#endif  /* WEBRTC_HOST_H_ */
