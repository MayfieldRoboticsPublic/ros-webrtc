#include <stdio.h>

#include <ros/ros.h>
#include <webrtc/base/ssladapter.h>
#include <webrtc/system_wrappers/interface/trace.h>

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
    ros::init(argc, argv, "host");

    ros::NodeHandle nh;

    ROS_INFO("loading config");
    Config config(Config::get(nh));

    if (!config.trace_file.empty()) {
        ROS_INFO(
            "setting webrtc trace file to %s w/ mask 0x%04x",
            config.trace_file.c_str(), config.trace_mask
        );
        webrtc::Trace::set_level_filter(config.trace_mask);
        webrtc::Trace::CreateTrace();
        webrtc::Trace::SetTraceFile(config.trace_file.c_str());
    }

    ROS_INFO("initializing ssl");
    if (!rtc::InitializeSSL()) {
        ROS_ERROR("ssl initialization failed");
        return 1;
    }

    ROS_INFO("creating host");
    HostFactory host_factory;
    host_factory.audio_src = config.microphone;
    for (size_t i = 0; i != config.cameras.size(); i++) {
        host_factory.video_srcs.push_back(config.cameras[i]);
    }
    for (size_t i = 0; i != config.ice_servers.size(); i++) {
        host_factory.ice_servers.push_back(config.ice_servers[i]);
    }
    host_factory.pc_constraints = config.pc_constraints;
    host_factory.pc_bond_timeout = config.pc_bond_timeout;
    host_factory.queue_sizes = config.queue_sizes;
    Host host = host_factory(nh);

    ROS_INFO("opening host ... ");
    if (!host.open(config.open_media_sources)) {
        ROS_INFO("host open failed");
        return 2;
    }
    ROS_INFO("opened host");

    ROS_INFO("scheduling host flush every %0.1f sec(s) ... ", config.flush_frequency);
    Flush flush(host);
    ros::Timer flush_timer = nh.createTimer(
        ros::Duration(config.flush_frequency), flush
    );
    flush_timer.start();

    ROS_INFO("start spinning");
    ros::spin();
    ROS_INFO("stop spinning");

    ROS_INFO("closing host ...");
    host.close();
    ROS_INFO("closed host");

    if (!config.trace_file.empty()) {
        ROS_INFO("resetting webrtc trace file");
        webrtc::Trace::SetTraceFile(NULL);
        webrtc::Trace::ReturnTrace();
    }

    return 0;
}
