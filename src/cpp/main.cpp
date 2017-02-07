#include <stdio.h>

#include <ros/ros.h>
#include <webrtc/base/ssladapter.h>
#include <webrtc/system_wrappers/include/trace.h>

#include "config.h"
#include "host.h"

#include <malloc.h>
#include <sys/resource.h>
#include <stdio.h>


struct Flush {

    Flush(Host& host) : host(host) {}

    void operator()(const ros::TimerEvent& event) {
        if (!ros::isShuttingDown()) {
            auto stats = host.flush();
            if (stats.reaped_data_messages) {
                ROS_INFO(
                    "flushed - reaped_data_messages=%zu",
                    stats.reaped_data_messages
                );
            }
        }
    }

    Host &host;

};

struct Reap {

    Reap(Host& host, double stale_threhold=30)
        : host(host),
          stale_threhold(stale_threhold) {
    }

    void operator()(const ros::TimerEvent& event) {
        if (!ros::isShuttingDown()) {
            auto stats = host.reap(stale_threhold);
            if (stats.deleted_connections) {
                ROS_INFO(
                    "reaped - deleted_connections=%zu",
                    stats.deleted_connections
                );
            }
        }
    }

    Host &host;
    double stale_threhold;

};

struct MemMonitor {

    MemMonitor(void*) {}

    void operator()(const ros::TimerEvent& event) {
        if (!ros::isShuttingDown()) {
            // Check our OOM score to
            //  see if we are running away with memory
            static FILE* proc_file;
            if(!proc_file) {
                char proc_path[128];
                int pidnum = (int)getpid();
                snprintf(proc_path,sizeof(proc_path)-1,
                         "/proc/%d/oom_score",pidnum);
                proc_file = fopen(proc_path,"r");
            }
            rewind(proc_file);
            fflush(proc_file);
            int oom_score;
            fscanf(proc_file,"%d",&oom_score);
            if(oom_score > 400) {
                ROS_FATAL(
                    "Memory went over limit (%d percent)",oom_score/10
                );
                abort();
            }
        }
    }
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

    struct rlimit corelimit = {0x70000000,0x70000000};
    if(setrlimit(RLIMIT_CORE,&corelimit) < 0) {
        ROS_ERROR("%s",strerror(errno));
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
    host_factory.pc_bond_connect_timeout = config.pc_bond_connect_timeout;
    host_factory.pc_bond_heartbeat_timeout = config.pc_bond_heartbeat_timeout;
    host_factory.queue_sizes = config.queue_sizes;
    Host host = host_factory(nh);

    ROS_INFO("opening host ... ");
    if (!host.open(config.open_media_sources)) {
        ROS_INFO("host open failed");
        return 2;
    }
    ROS_INFO("opened host");

    Flush flush(host);
    ros::Timer flush_timer = nh.createTimer(
        ros::Duration(config.flush_frequency), flush
    );
    if (config.flush_frequency != 0) {
        ROS_INFO("scheduling host flush every %0.1f sec(s) ... ", config.flush_frequency);
        flush_timer.start();
    }

    Reap reap(host);
    ros::Timer reap_timer = nh.createTimer(
        ros::Duration(config.reap_frequency), reap
    );
    if (config.reap_frequency != 0) {
        ROS_INFO("scheduling host reap every %0.1f sec(s) ... ", config.reap_frequency);
        reap_timer.start();
    }

    MemMonitor monitor(NULL);
    ros::Timer mem_timer = nh.createTimer(
        ros::Duration(1.0), monitor
    );
    mem_timer.start();

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
