#ifndef WEBRTC_DEVICE_H_
#define WEBRTC_DEVICE_H_

#include <ros_webrtc/Connect.h>
#include <ros_webrtc/Data.h>
#include <ros_webrtc/Disconnect.h>
#include <ros_webrtc/IceCandidate.h>
#include <ros_webrtc/SdpOfferAnswer.h>
#include <ros_webrtc/Sessions.h>
#include <talk/app/webrtc/peerconnectioninterface.h>

#include "media_constraints.h"
#include "session.h"


struct DeviceVideoSource {

    enum Type {
        NoneType = 0,
        DeviceType,
        ROSTopicType
    };

    DeviceVideoSource();

    DeviceVideoSource(
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

};

struct DeviceAudioSource {

    DeviceAudioSource();

    DeviceAudioSource(
        const std::string& label,
        const MediaConstraints& constraints,
        bool publish = false
    );

    std::string label;

    MediaConstraints constraints;

    bool publish;

};

struct DeviceFactory {

    Device operator()();

    std::vector<DeviceVideoSource> video_srcs;

    DeviceAudioSource audio_src;

    MediaConstraints session_constraints;

    std::vector<webrtc::PeerConnectionInterface::IceServer> ice_servers;

};

class Device {

public:

    Device(
        const std::vector<DeviceVideoSource>& video_srcs,
        const DeviceAudioSource& audio_src,
        const MediaConstraints& session_constraints,
        const std::vector<webrtc::PeerConnectionInterface::IceServer>& ice_servers
    );

    Device(const Device& other);

    ~Device();

    const std::string& id() const;

    bool open();

    bool is_open() const;

    void close();

    SessionPtr begin_session(
        const std::string& peer_id,
        const MediaConstraints& sdp_constraints,
        const std::vector<ros_webrtc::DataChannel>& data_channels,
        const std::map<std::string, std::string>& service_names
    );

    bool end_session(const std::string& peer_id);

    typedef std::list<SessionPtr> Sessions;

    typedef std::list<SessionConstPtr> SessionsConst;

    SessionPtr session(const std::string& peer_id);

    SessionConstPtr session(const std::string& peer_id) const;

    Sessions& sessions();

    const Sessions& sessions() const;

    struct Flush {

        size_t reaped_data_messages;

        Flush& operator += (const Session::Flush & rhs);

    };

    Flush flush();

private:

    bool _create_pc_factory();

    bool _open_local_stream();

    void _close_local_stream();

    bool _open_servers();

    void _close_servers();

    bool _serve_connect(ros::ServiceEvent<ros_webrtc::Connect::Request, ros_webrtc::Connect::Response>& event);

    bool _serve_disconnect(ros::ServiceEvent<ros_webrtc::Disconnect::Request, ros_webrtc::Disconnect::Response>& event);

    bool _serve_ice_candidate(ros::ServiceEvent<ros_webrtc::IceCandidate::Request, ros_webrtc::IceCandidate::Response>& event);

    bool _serve_sdp_offer_answer(ros::ServiceEvent<ros_webrtc::SdpOfferAnswer::Request, ros_webrtc::SdpOfferAnswer::Response>& event);

    bool _serve_sessions(ros::ServiceEvent<ros_webrtc::Sessions::Request, ros_webrtc::Sessions::Response>& event);

    void _handle_send(const ros_webrtc::DataConstPtr& msg);

    class SessionObserver : public Session::Observer {

    public:

        SessionObserver(Device& instance_, SessionPtr session_);

        ~SessionObserver();

    private:

        Device& instance;

        SessionPtr session;

    // Session::Observer

    public:

        void on_connection_change(webrtc::PeerConnectionInterface::IceConnectionState state);

    };

    ros::NodeHandle _nh;

    std::vector<DeviceVideoSource> _video_srcs;

    DeviceAudioSource _audio_src;

    MediaConstraints _session_constraints;

    std::vector<webrtc::PeerConnectionInterface::IceServer> _ice_servers;

    rtc::scoped_ptr<rtc::Thread> _signaling_thd;

    rtc::scoped_ptr<rtc::Thread> _worker_thd;

    rtc::scoped_refptr<webrtc::PeerConnectionFactoryInterface> _pc_factory;

    rtc::scoped_refptr<webrtc::MediaStreamInterface> _local_stream;

    std::string _audio_label;

    AudioSinkPtr _audio_sink;

    std::list<VideoRendererPtr> _video_renderers;

    ros::V_ServiceServer _rsrvs;

    ros::V_Subscriber _rsubs;

    Sessions _sessions;

    ros::Publisher _dc_rpub;

};

typedef boost::shared_ptr<Device> DevicePtr;

#endif  /* WEBRTC_DEVICE_H_ */
