#include <stdio.h>

#include <ros/ros.h>
#include <webrtc/base/ssladapter.h>

#include "config.h"
#include "host.h"


struct Flush {

    Flush(Host& host) : host(host) {}

    void operator()(const ros::TimerEvent& event) {
        ROS_INFO("flushing");
        host.flush();
    }

    Host &host;

};

int main(int argc, char **argv) {
    ROS_INFO("initializing ros");
    ros::init(argc, argv, "webrtc", ros::init_options::AnonymousName);

    ros::NodeHandle nh("~");

    ROS_INFO("loading config");
    Config config(Config::get(nh));

    ROS_INFO("initializing ssl");
    if (!rtc::InitializeSSL()) {
        ROS_ERROR("ssl initialization failed");
        return 1;
    }

    ROS_INFO("creating host");
    HostFactory host_factory;
    host_factory.audio_src = config.microphone;
    for(size_t i = 0; i != config.cameras.size(); i++) {
        host_factory.video_srcs.push_back(config.cameras[i]);
    }
    for(size_t i = 0; i != config.ice_servers.size(); i++) {
        host_factory.ice_servers.push_back(config.ice_servers[i]);
    }
    host_factory.session_constraints = config.session_constraints;
    Host host = host_factory(nh);

    ROS_INFO("opening host ... ");
    if (!host.open()) {
        ROS_INFO("host open failed");
        return 2;
    }
    ROS_INFO("opened host");

    ROS_INFO("scheduling host flush every %0.1f sec(s) ... ", config.flush_frequency);
    Flush flush(host);
    ros::Timer flush_timer = nh.createTimer(
        ros::Duration(config.flush_frequency), flush
    );

    ROS_INFO("start spinning");
    ros::spin();
    ROS_INFO("stop spinning");

    ROS_INFO("closing host ...");
    host.close();
    ROS_INFO("closed host");

    return 0;
}
