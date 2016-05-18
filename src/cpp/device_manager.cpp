#include "device_manager.h"

#include <algorithm>

#include <ros/ros.h>
#include <talk/media/webrtc/webrtcvideocapturer.h>
#include <talk/media/webrtc/webrtcvideocapturerfactory.h>
#include <webrtc/system_wrappers/interface/critical_section_wrapper.h>
#include <webrtc/system_wrappers/interface/thread_wrapper.h>

#include "video_capture.h"

// ROSWebRtcVcmFactory

/**
 * \brief Factory for exposing ROS video capture module and devices to cricket::WebRtcVideoCapturer.
 */
class ROSWebRtcVcmFactory : public cricket::WebRtcVcmFactoryInterface {

public:

    ROSWebRtcVcmFactory(
        const cricket::Device &device,
        ROSVideoCaptureTopicInfoConstPtr topics,
        VideoCaptureModuleRegistryPtr module_reg) :
            _device_id(device.id),
            _topics(topics),
            _module_reg(module_reg) {
    }

    virtual ~ROSWebRtcVcmFactory() {}

private:

    std::string _device_id;
    ROSVideoCaptureTopicInfoConstPtr _topics;
    VideoCaptureModuleRegistryPtr _module_reg;

// cricket::WebRtcVcmFactoryInterface

public:

    virtual webrtc::VideoCaptureModule* Create(int id, const char* device) {
        rtc::scoped_refptr<webrtc::VideoCaptureModule> module(
            ROSVideoCaptureModule::Create(id, device)
        );
        if (_module_reg != NULL) {
            _module_reg->add(_device_id, module.get());
        }
        return module.release();
    }

    virtual webrtc::VideoCaptureModule::DeviceInfo* CreateDeviceInfo(int id) {
        std::auto_ptr<webrtc::VideoCaptureModule::DeviceInfo> devices(
            ROSVideoCaptureModule::CreateDeviceInfo(id)
        );
        if (devices.get() == NULL)
            return NULL;
        // HACK: need to customize initialization of ROSVideoCaptureDeviceInfo
        if (!dynamic_cast<ROSVideoCaptureDeviceInfo *>(devices.get())->init(_topics))
            return NULL;
        return devices.release();
    }

    virtual void DestroyDeviceInfo(webrtc::VideoCaptureModule::DeviceInfo* info) {
        if (info) {
            delete info;
        }
    }

};

// ROSWebRtcVideoDeviceCapturerFactory

/**
 * \brief Factory for exposing ROS video capturers to cricket::DeviceManager.
 */
class ROSWebRtcVideoDeviceCapturerFactory : public cricket::WebRtcVideoDeviceCapturerFactory {

public:

    ROSWebRtcVideoDeviceCapturerFactory(
        ROSVideoCaptureTopicInfoConstPtr topics,
        VideoCaptureModuleRegistryPtr module_reg) :
            _topics(topics),
            _module_reg(module_reg) {
    }

private:

    ROSVideoCaptureTopicInfoConstPtr _topics;
    VideoCaptureModuleRegistryPtr _module_reg;

// cricket::WebRtcVideoDeviceCapturerFactory

public:

    virtual cricket::VideoCapturer* Create(const cricket::Device& device) {
        rtc::scoped_ptr<cricket::WebRtcVideoCapturer> capturer(
            new cricket::WebRtcVideoCapturer(
                new ROSWebRtcVcmFactory(device, _topics, _module_reg)
            )
        );
        if (!capturer->Init(device)) {
            return NULL;
        }
        return capturer.release();
    }

};

// ROSDeviceManager

ROSDeviceManager::ROSDeviceManager(
    ROSVideoCaptureTopicInfoConstPtr video_capture_topics) :
        ROSDeviceManager(video_capture_topics, NULL) {
}

ROSDeviceManager::ROSDeviceManager(
    ROSVideoCaptureTopicInfoConstPtr video_capture_topics,
    VideoCaptureModuleRegistryPtr video_capture_module_reg) :
        _video_capture_topics(video_capture_topics) {
    SetVideoDeviceCapturerFactory(
        new ROSWebRtcVideoDeviceCapturerFactory(
            video_capture_topics, video_capture_module_reg
        )
    );
    set_watcher(new cricket::DeviceWatcher(this));
}

ROSDeviceManager::~ROSDeviceManager() {
}

bool ROSDeviceManager::GetVideoCaptureDevices(std::vector<cricket::Device>* devs) {
    devs->clear();
    for (size_t i = 0; i < _video_capture_topics->size(); i++) {
        ROS_INFO(
            "ROS video capture device '%s' w/ id %zu",
            _video_capture_topics->get(i).name.c_str(), i
        );
        devs->push_back(cricket::Device(_video_capture_topics->get(i).name, i));
    }
    return true;
}

bool ROSDeviceManager::GetAudioDevices(bool input, std::vector<cricket::Device>* devs) {
    return true;
}
