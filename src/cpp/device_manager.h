#ifndef ROS_WEBRTC_DEVICE_MANAGER_H_
#define ROS_WEBRTC_DEVICE_MANAGER_H_

#include <talk/media/devices/devicemanager.h>

#include "video_capture.h"

/**
 * \brief Represents ROS topics as media (video, audio, etc) devices.
 */
class ROSDeviceManager : public cricket::DeviceManager {

public:

    ROSDeviceManager(
        ROSVideoCaptureTopicInfoConstPtr video_capture_topics
    );

    ROSDeviceManager(
        ROSVideoCaptureTopicInfoConstPtr video_capture_topics,
        VideoCaptureModuleRegistryPtr video_capture_module_reg
    );

    ROSDeviceManager();

    virtual ~ROSDeviceManager();

private:

    ROSVideoCaptureTopicInfoConstPtr _video_capture_topics;

// cricket::DeviceManager

public:

    virtual bool GetVideoCaptureDevices(std::vector<cricket::Device>* devs);

protected:

    virtual bool GetAudioDevices(bool input, std::vector<cricket::Device>* devs);

};

#endif /* ROS_WEBRTC_DEVICE_MANAGER_H_ */
