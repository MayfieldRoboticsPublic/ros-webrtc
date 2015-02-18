#ifndef ROS_WEBRTC_CONFIG_H_
#define ROS_WEBRTC_CONFIG_H_

#include <ros/ros.h>
#include <talk/app/webrtc/peerconnectioninterface.h>

#include "host.h"
#include "media_constraints.h"

/*
 * @class Config
 * @brief Configuration settings read from ROS params.
 */
class Config {

public:

    /**
     * @brief Factory function used to load configuration settings from ROS params.
     * @return Instance representing loaded ROS params.
     *
     * The ROS params look something like:
     *
     * @code{.yaml}

       cameras:
        downward:
          name: ros:///downward_facing_camera/image_raw
          label: downward
        upward:
          name: ros:///upward_facing_camera/image_raw
          label: upward
       session:
         constraints:
           optional:
             DtlsSrtpKeyAgreement: "true"
       ice_servers:
       - uri: stun:stun.services.mozilla.com:3478
       - uri: stun:stun.l.google.com:19302

     * @endcode
     */
    static Config get();

    /**
     * @brief Persists configuration settings to ROS params.
     */
    void set();

    std::vector<VideoSource> cameras; /*! Video sources. */

    AudioSource microphone; /*! Single audio source. */

    MediaConstraints session_constraints; /*! Session media constraints. */

    typedef webrtc::PeerConnectionInterface::IceServers IceServers;

    IceServers ice_servers; /*! Servers to use for ICE. */

    int flush_frequency; /*! Number of seconds between device/session flushes . */

private:

    static bool _get(ros::NodeHandle& nh, const std::string& root, VideoSource& value);

    static void _set(ros::NodeHandle& nh, const std::string& root, const VideoSource& value);

    static bool _get(ros::NodeHandle& nh, const std::string& root, AudioSource& value);

    static void _set(ros::NodeHandle& nh, const std::string& root, const AudioSource& value);

    static bool _get(ros::NodeHandle& nh, const std::string& root, MediaConstraints& value);

    static void _set(ros::NodeHandle& nh, const std::string& root, const MediaConstraints& value);

    static bool _get(ros::NodeHandle& nh, const std::string& root, webrtc::PeerConnectionInterface::IceServer& value);

    static void _set(ros::NodeHandle& nh, const std::string& root, const webrtc::PeerConnectionInterface::IceServer& value);

};

#endif /* ROS_WEBRTC_CONFIG_H_ */
