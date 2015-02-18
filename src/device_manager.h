#ifndef ROS_WEBRTC_DEVICE_MANAGER_H_
#define ROS_WEBRTC_DEVICE_MANAGER_H_

#include <talk/media/devices/devicemanager.h>

class ROSDeviceManager : public cricket::DeviceManager {

public:

    ROSDeviceManager();

    virtual ~ROSDeviceManager();

// cricket::DeviceManager

public:

    virtual bool GetVideoCaptureDevices(std::vector<cricket::Device>* devs);

protected:

    virtual bool GetAudioDevices(bool input, std::vector<cricket::Device>* devs);

};

#endif /* ROS_WEBRTC_DEVICE_MANAGER_H_ */
