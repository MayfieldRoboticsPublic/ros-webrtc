#ifndef WEBRTC_VIDEO_CAPTURE_H_
#define WEBRTC_VIDEO_CAPTURE_H_

#include <queue>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <talk/media/webrtc/webrtcvideocapturer.h>
#include <webrtc/modules/video_capture/device_info_impl.h>
#include <webrtc/modules/video_capture/video_capture_impl.h>
#include <webrtc/system_wrappers/interface/critical_section_wrapper.h>
#include <webrtc/system_wrappers/interface/thread_wrapper.h>


class VideoCaptureDeviceInfo : public webrtc::videocapturemodule::DeviceInfoImpl {

public:

    VideoCaptureDeviceInfo(const int32_t id);

    virtual ~VideoCaptureDeviceInfo();

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

class VideoCaptureModule : public webrtc::videocapturemodule::VideoCaptureImpl {

public:

    VideoCaptureModule(int32_t id);

    virtual ~VideoCaptureModule();

    int32_t init(const char* deviceUniqueIdUTF8);

private:

    struct Frame {

        std::vector<uint8_t> buffer;

        size_t width;

    };

    static bool _capture(void*);

    bool _capture_image();

    void _enqueue_image(const sensor_msgs::ImageConstPtr& msg);

    webrtc::VideoCaptureCapability _capability;

    webrtc::CriticalSectionWrapper* _capture_cs;

    bool _capturing;

    std::string _topic;

    ros::Subscriber _subscriber;

    ros::NodeHandle _nh;

    ros::CallbackQueue _cb_q;

    std::string _frame_encoding;

    typedef std::queue<Frame> FrameQueue;

    FrameQueue _frames;

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

class WebRtcVcmFactory : public cricket::WebRtcVcmFactoryInterface {

public:

    virtual ~WebRtcVcmFactory();

// cricket::WebRtcVcmFactoryInterface

public:

    virtual webrtc::VideoCaptureModule* Create(int id, const char* device);

    virtual webrtc::VideoCaptureModule::DeviceInfo* CreateDeviceInfo(int id);

    virtual void DestroyDeviceInfo(webrtc::VideoCaptureModule::DeviceInfo* info);

};

#endif /* WEBRTC_VIDEO_CAPTURE_H_ */
