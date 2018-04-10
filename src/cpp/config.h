#ifndef ROS_WEBRTC_CONFIG_H_
#define ROS_WEBRTC_CONFIG_H_

#include <ros/ros.h>
#include <webrtc/api/peerconnectioninterface.h>
#include <webrtc/common_types.h>

#include "host.h"
#include "media_constraints.h"

/**
 * \brief Configuration settings read from ROS params.
 */
class Config {

public:

    /**
     * \brief Factory function used to load configuration settings from ROS params.
     * \return Instance representing loaded ROS params.
     *
     * The ROS params look something like:
     *
     * \code{.yaml}

       cameras:
        downward:
          # video device is ROS sensor_msgs/Image topic
          name: ros:///downward_looking_camera/image_raw
          label: downward
        upward:
          # video device is file-system path
          name: id:///dev/upward_looking_camera
          label: upward
          constraints:
              mandatory:
                maxWidth: "640"
                minWidth: "640"
                maxHeight: "480"
                minHeight: "480"
        wayward:
          # video device is system dependent name (e.g. Linux 2.6+ its `cat /sys/class/video4linux/video{#}/name`)
          name: HD Camera
          label: wayward
       peer_connection:
        connect_timeout: 10.0
        heartbeat_timeout: 4.0
        constraints:
          optional:
          DtlsSrtpKeyAgreement: "true"
       ice_servers:
       - uri: stun:stun.services.mozilla.com:3478
       - uri: stun:stun.l.google.com:19302
       trace:
        file: /tmp/ros_webrtc.trace
        filter: all
       queue_sizes:
        audio: 1000
        video: 1000
        data: 1000
       open_media_sources: true

     * \endcode
     */
    static Config get(ros::NodeHandle& nh);

    /**
     * \brief Persists configuration settings to ROS params.
     */
    void set();

    std::vector<VideoSource> cameras; /*! Video sources. */

    AudioSource microphone; /*! Single audio source. */

    double pc_bond_connect_timeout; /*! Peer connection bond connect timeout, or 0 for no bonding. */

    double pc_bond_heartbeat_timeout; /*! Peer connection bond heartbeat timeout, or 0 for no bonding. */

    MediaConstraints pc_constraints; /*! Peer connection media constraints. */

    typedef webrtc::PeerConnectionInterface::IceServers IceServers;

    IceServers default_ice_servers; /*! Servers to use for ICE. */

    double flush_frequency; /*! Number of seconds between host/session flushes. */

    double reap_frequency; /*! Number of seconds between host/connection reaps. */

    std::string trace_file; /*! Write WebRTC traces to this file. */

    uint32_t trace_mask; /*! Filter WebRTC traces using this mask. */

    QueueSizes queue_sizes; /*! Sizes of audio, video and data publisher/subscriber queues. */

    bool open_media_sources; /*! Open media sources on start. */

private:

    static bool _get(ros::NodeHandle& nh, const std::string& root, VideoSource& value);

    static bool _get(ros::NodeHandle& nh, const std::string& root, AudioSource& value);

    static bool _get(ros::NodeHandle& nh, const std::string& root, MediaConstraints& value);

    static bool _get(ros::NodeHandle& nh, XmlRpc::XmlRpcValue& root, webrtc::PeerConnectionInterface::IceServer& value);

    static bool _get(ros::NodeHandle& nh, const std::string& root, QueueSizes& value);

    typedef std::map<std::string, webrtc::TraceLevel> TraceLevels;

    static TraceLevels _trace_levels;

};

#endif /* ROS_WEBRTC_CONFIG_H_ */
