#ifndef WEBRTC_CONFIG_H_
#define WEBRTC_CONFIG_H_

#include <ros/ros.h>
#include <talk/app/webrtc/peerconnectioninterface.h>

#include "device.h"
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
          name: "Loopback video device 0"
          label: downward
        upward:
          name: "Loopback video device 1"
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

    std::vector<DeviceVideoSource> cameras; /*! Video input devices. */

    DeviceAudioSource microphone; /*! Single audio input device. */

    MediaConstraints session_constraints; /*! Session media constraints. */

    typedef webrtc::PeerConnectionInterface::IceServers IceServers;

    IceServers ice_servers; /*! Servers to use for ICE. */

private:

    static bool _get(ros::NodeHandle& nh, const std::string& root, DeviceVideoSource& value);

    static void _set(ros::NodeHandle& nh, const std::string& root, const DeviceVideoSource& value);

    static bool _get(ros::NodeHandle& nh, const std::string& root, DeviceAudioSource& value);

    static void _set(ros::NodeHandle& nh, const std::string& root, const DeviceAudioSource& value);

    static bool _get(ros::NodeHandle& nh, const std::string& root, MediaConstraints& value);

    static void _set(ros::NodeHandle& nh, const std::string& root, const MediaConstraints& value);

    static bool _get(ros::NodeHandle& nh, const std::string& root, webrtc::PeerConnectionInterface::IceServer& value);

    static void _set(ros::NodeHandle& nh, const std::string& root, const webrtc::PeerConnectionInterface::IceServer& value);

};

#endif /* WEBRTC_CONFIG_H_ */
