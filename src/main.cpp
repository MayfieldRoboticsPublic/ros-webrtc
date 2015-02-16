#include <stdio.h>

#include <ros/ros.h>
#include <signal.h>
#include <webrtc/base/ssladapter.h>

#include "config.h"
#include "device.h"


struct Flush {

    Flush(Device& device_) : device(device_) {}

    void operator () (const ros::WallTimerEvent& event) {
        device.flush();
    }

    Device &device;

};

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
    device_factory.audio_src = config.microphone;
    for(size_t i = 0; i != config.cameras.size(); i++) {
        device_factory.video_srcs.push_back(config.cameras[i]);
    }
    for(size_t i = 0; i != config.ice_servers.size(); i++) {
        device_factory.ice_servers.push_back(config.ice_servers[i]);
    }
    device_factory.session_constraints = config.session_constraints;
    Device device(device_factory());

    ROS_INFO_STREAM("opening device ... ");
    if (!device.open()) {
        ROS_INFO_STREAM("device open failed");
        return 2;
    }
    ROS_INFO_STREAM("opened device");

    ROS_INFO_STREAM("scheduling device flush every " << config.flush_frequency << " sec(s) ... ");
    ros::NodeHandle nh;
    Flush flush(device);
    ros::WallTimer flush_timer = nh.createWallTimer(
        ros::WallDuration(config.flush_frequency), flush, false
    );

    ROS_INFO_STREAM("start spinning");
    ros::spin();
    ROS_INFO_STREAM("stop spinning");

    ROS_INFO_STREAM("closing device ...");
    device.close();
    ROS_INFO_STREAM("closed device");

    return 0;
}
