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

    ROSWebRtcVcmFactory(ROSVideoCaptureTopicInfoConstPtr topics) : _topics(topics) {}

    virtual ~ROSWebRtcVcmFactory() {}

private:

    ROSVideoCaptureTopicInfoConstPtr _topics;

// cricket::WebRtcVcmFactoryInterface

public:

    virtual webrtc::VideoCaptureModule* Create(int id, const char* device) {
        return ROSVideoCaptureModule::Create(id, device);
    }

    virtual webrtc::VideoCaptureModule::DeviceInfo* CreateDeviceInfo(int id) {
        std::auto_ptr<webrtc::VideoCaptureModule::DeviceInfo> devices(
            ROSVideoCaptureModule::CreateDeviceInfo(id)
        );
        if (devices.get() == NULL)
            return NULL;
        if (!dynamic_cast<ROSVideoCaptureDeviceInfo *>(devices.get())->init(_topics))
            return NULL;
        return devices.release();
    }

    virtual void DestroyDeviceInfo(webrtc::VideoCaptureModule::DeviceInfo* info) {
        // FIXME: do nothing?
    }

};

// ROSWebRtcVideoDeviceCapturerFactory

class ROSWebRtcVideoDeviceCapturerFactory : public cricket::WebRtcVideoDeviceCapturerFactory {

public:

    ROSWebRtcVideoDeviceCapturerFactory(
        ROSVideoCaptureTopicInfoConstPtr topics
        ) : _topics(topics) {
    }

private:

    ROSVideoCaptureTopicInfoConstPtr _topics;

// cricket::WebRtcVideoDeviceCapturerFactory

public:

    virtual cricket::VideoCapturer* Create(const cricket::Device& device) {
        rtc::scoped_ptr<cricket::WebRtcVideoCapturer> capturer(
            new cricket::WebRtcVideoCapturer(new ROSWebRtcVcmFactory(_topics))
        );
        if (!capturer->Init(device)) {
            return NULL;
        }
        return capturer.release();
    }

};

// ROSDeviceManager

ROSDeviceManager::ROSDeviceManager(
    ROSVideoCaptureTopicInfoConstPtr video_capture_topics
    ) : _video_capture_topics(video_capture_topics) {
    SetVideoDeviceCapturerFactory(
        new ROSWebRtcVideoDeviceCapturerFactory(video_capture_topics)
    );
    set_watcher(new cricket::DeviceWatcher(this));
}

ROSDeviceManager::~ROSDeviceManager() {
}

bool ROSDeviceManager::GetVideoCaptureDevices(std::vector<cricket::Device>* devs) {
    devs->clear();
    for (size_t i = 0; i < _video_capture_topics->size(); i++) {
        ROS_INFO(
            "ROS video capture device '%s'",
            _video_capture_topics->get(i).name.c_str()
        );
        devs->push_back(cricket::Device(_video_capture_topics->get(i).name, i));
    }
    return true;
}

bool ROSDeviceManager::GetAudioDevices(bool input, std::vector<cricket::Device>* devs) {
    return true;
}
