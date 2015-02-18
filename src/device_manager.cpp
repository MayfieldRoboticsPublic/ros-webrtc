#include "device_manager.h"

#include <algorithm>

#include <ros/ros.h>
#include <talk/media/webrtc/webrtcvideocapturer.h>
#include <talk/media/webrtc/webrtcvideocapturerfactory.h>
#include <webrtc/system_wrappers/interface/critical_section_wrapper.h>
#include <webrtc/system_wrappers/interface/thread_wrapper.h>

#include "video_capture.h"

// ROSWebRtcVcmFactory

class ROSWebRtcVcmFactory : public cricket::WebRtcVcmFactoryInterface {

public:

    virtual ~ROSWebRtcVcmFactory() {
    }

// cricket::WebRtcVcmFactoryInterface

public:

    virtual webrtc::VideoCaptureModule* Create(int id, const char* device)  {
        return ROSVideoCaptureModule::Create(id, device);
    }

    virtual webrtc::VideoCaptureModule::DeviceInfo* CreateDeviceInfo(int id)  {
        return ROSVideoCaptureModule::CreateDeviceInfo(id);
    }

    virtual void DestroyDeviceInfo(webrtc::VideoCaptureModule::DeviceInfo* info) {
        // FIXME: do nothing?
    }

};

// ROSWebRtcVideoDeviceCapturerFactory

class ROSWebRtcVideoDeviceCapturerFactory : public cricket::WebRtcVideoDeviceCapturerFactory {

// cricket::WebRtcVideoDeviceCapturerFactory

public:

    virtual cricket::VideoCapturer* Create(const cricket::Device& device) {
        rtc::scoped_ptr<cricket::WebRtcVideoCapturer> capturer(
            new cricket::WebRtcVideoCapturer(new ROSWebRtcVcmFactory())
        );
        if (!capturer->Init(device)) {
            return NULL;
        }
        return capturer.release();
    }

};

// ROSDeviceManager

ROSDeviceManager::ROSDeviceManager() {
    SetVideoDeviceCapturerFactory(new ROSWebRtcVideoDeviceCapturerFactory());
    set_watcher(new cricket::DeviceWatcher(this));
}

ROSDeviceManager::~ROSDeviceManager() {
}

bool ROSDeviceManager::GetVideoCaptureDevices(std::vector<cricket::Device>* devs) {
    devs->clear();
    ros::master::V_TopicInfo topics;
    if (!ros::master::getTopics(topics)) {
        ROS_WARN_STREAM("failed to get topics");
        return false;
    }
    for (size_t i = 0; i < topics.size(); i++) {
        const ros::master::TopicInfo& topic = topics[i];
        if (topic.datatype == "sensor_msgs/Image") {
            devs->push_back(cricket::Device(
                topics[i].name, topics[i].name
            ));
        }
    }
    return true;
}

bool ROSDeviceManager::GetAudioDevices(bool input, std::vector<cricket::Device>* devs) {
    return true;
}
