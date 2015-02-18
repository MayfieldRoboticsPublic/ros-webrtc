#ifndef ROS_WEBRTC_VIDEO_CAPTURE_H_
#define ROS_WEBRTC_VIDEO_CAPTURE_H_

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <webrtc/modules/video_capture/device_info_impl.h>
#include <webrtc/modules/video_capture/video_capture_impl.h>
#include <webrtc/system_wrappers/interface/critical_section_wrapper.h>
#include <webrtc/system_wrappers/interface/thread_wrapper.h>


class ROSVideoCaptureModule : public webrtc::videocapturemodule::VideoCaptureImpl {

public:

    ROSVideoCaptureModule(int32_t id);

    virtual ~ROSVideoCaptureModule();

    int32_t init(const char* deviceUniqueIdUTF8);

private:

    static bool _capture_thread(void*);

    bool _capture_poll();

    void _image_callback(const sensor_msgs::ImageConstPtr& msg);

    webrtc::VideoCaptureCapability _capability;

    webrtc::CriticalSectionWrapper* _capture_cs;

    bool _capturing;

    std::string _topic;

    ros::Subscriber _subscriber;

    ros::NodeHandle _nh;

    ros::CallbackQueue _image_q;

    webrtc::ThreadWrapper* _capture_thd;

// webrtc::videocapturemodule::VideoCaptureImpl

public:

    static webrtc::VideoCaptureModule* Create(const int32_t id, const char* deviceUniqueIdUTF8);

    static DeviceInfo* CreateDeviceInfo(const int32_t id);

    virtual int32_t StartCapture(const webrtc::VideoCaptureCapability& capability);

    virtual int32_t StopCapture();

    virtual bool CaptureStarted();

    virtual int32_t CaptureSettings(webrtc::VideoCaptureCapability& settings);

};


class ROSVideoCaptureDeviceInfo : public webrtc::videocapturemodule::DeviceInfoImpl {

public:

    ROSVideoCaptureDeviceInfo(const int32_t id);

    virtual ~ROSVideoCaptureDeviceInfo();

private:

    struct topic_name_less_than {

        inline bool operator() (const ros::master::TopicInfo& a, const ros::master::TopicInfo& b) {
           return (a.name < b.name);
       }

    };

// webrtc::VideoCaptureModule::DeviceInfo

public:

    virtual uint32_t NumberOfDevices();

    virtual int32_t GetDeviceName(
        uint32_t deviceNumber,
        char* deviceNameUTF8,
        uint32_t deviceNameLength,
        char* deviceUniqueIdUTF8,
        uint32_t deviceUniqueIdUTF8Length,
        char* productUniqueIdUTF8=0,
        uint32_t productUniqueIdUTF8Length=0
    );

    virtual int32_t CreateCapabilityMap (const char* deviceUniqueIdUTF8);

    virtual int32_t DisplayCaptureSettingsDialogBox(
        const char* deviceUniqueIdUTF8,
        const char* dialogTitleUTF8,
        void* parentWindow,
        uint32_t positionX,
        uint32_t positionY
    );

    int32_t Init();

};

#endif /* VIDEO_CAPTURE_H_ */
