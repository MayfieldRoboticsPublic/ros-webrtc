#include <stdio.h>

#include <ros/ros.h>
#include <signal.h>
#include <webrtc/base/ssladapter.h>

#include "config.h"
#include "device.h"

int main(int argc, char **argv) {
    ROS_INFO_STREAM("initializing ros");
    ros::init(argc, argv, "webrtc", ros::init_options::AnonymousName);

    ROS_INFO_STREAM("loading config");
    Config config(Config::get());

    ROS_INFO_STREAM("initializing ssl");
    if (!rtc::InitializeSSL()) {
        ROS_ERROR_STREAM("ssl initialization failed");
        return 1;
    }

    ROS_INFO_STREAM("creating device");
    DeviceFactory device_factory;
    device_factory.audio_label = config.microphone.label;
    device_factory.audio_constraints = config.microphone.constraints;
    for(Config::Cameras::iterator i = config.cameras.begin(); i != config.cameras.end(); i++) {
        device_factory.add_video(
            (*i).second.name,
            (*i).second.label,
            (*i).second.constraints
        );
    }
    for(Config::IceServers::iterator i = config.ice_servers.begin(); i != config.ice_servers.end(); i++) {
        device_factory.add_ice_server(*(i));
    }
    device_factory.session_constraints = config.session_constraints;
    Device device(device_factory());

    ROS_INFO_STREAM("opening device ... ");
    if (!device.open()) {
        ROS_INFO_STREAM("device open failed");
        return 2;
    }
    ROS_INFO_STREAM("opened device");

    ROS_INFO_STREAM("start spinning");
    ros::spin();
    ROS_INFO_STREAM("stop spinning");

    ROS_INFO_STREAM("closing device ...");
    device.close();
    ROS_INFO_STREAM("closed device");

    return 0;
}
