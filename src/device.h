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

class DeviceFactory {

public:

    void add_video(const std::string& name, const std::string& label, const MediaConstraints& constraints);

    void add_ice_server(const webrtc::PeerConnectionInterface::IceServer& ice_server);

    Device operator()();

    std::vector<std::string> video_srcs;

    std::vector<std::string> video_labels;

    std::vector<MediaConstraints> video_constraints;

    std::string audio_label;

    MediaConstraints audio_constraints;

    MediaConstraints session_constraints;

    std::vector<webrtc::PeerConnectionInterface::IceServer> ice_servers;

};

class Device {

public:

    Device(
        const std::vector<std::string>& video_srcs,
        const std::vector<std::string>& video_labels,
        const std::vector<MediaConstraints>& video_constraints,
        const MediaConstraints& audio_constraints,
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

private:

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

    std::vector<std::string> _video_srcs;

    std::vector<std::string> _video_labels;

    std::vector<MediaConstraints> _video_constraints;

    MediaConstraints _audio_constraints;

    MediaConstraints _session_constraints;

    std::vector<webrtc::PeerConnectionInterface::IceServer> _ice_servers;

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
