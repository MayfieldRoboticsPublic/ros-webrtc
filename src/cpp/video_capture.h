#ifndef ROS_WEBRTC_VIDEO_CAPTURE_H_
#define ROS_WEBRTC_VIDEO_CAPTURE_H_

#include <map>
#include <memory>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <webrtc/base/refcount.h>
#include <webrtc/base/thread.h>
#include <webrtc/media/base/videocapturer.h>
#include <webrtc/media/engine/webrtcvideocapturerfactory.h>
#include <webrtc/modules/video_capture/device_info_impl.h>
#include <webrtc/modules/video_capture/video_capture.h>
#include <webrtc/modules/video_capture/video_capture_impl.h>
#include <webrtc/system_wrappers/include/critical_section_wrapper.h>

#ifdef USE_MADMUX
#include <madmux/madmux.h>
#include "geocam_cfg.h"
#endif

/**
 * \brief Maps cricket::VideoCapturer to their associated cricket::VideoCaptureModule.
 */
class VideoCaptureModuleRegistry {

public:

    void add(cricket::VideoCapturer *capturer, webrtc::VideoCaptureModule *module);

    void add(const std::string& id, webrtc::VideoCaptureModule *module);

    bool remove(cricket::VideoCapturer *capturer);

    bool remove(const std::string& id);

    void remove_all();

    webrtc::VideoCaptureModule* find(cricket::VideoCapturer *capturer) const;

    webrtc::VideoCaptureModule* find(const std::string& id) const;

    std::vector<std::string> ids() const;

private:

    typedef rtc::scoped_refptr<webrtc::VideoCaptureModule> module_type;

    std::map<std::string, module_type>_modules;

};

typedef std::shared_ptr<VideoCaptureModuleRegistry> VideoCaptureModuleRegistryPtr;

#ifdef USE_MADMUX

/**
 * \brief Allows capturing frames from GEO semi cameras
 *
 * They use their own API (mxuvc) to get camera frames.
 */
class GeoVideoCaptureModule : public webrtc::videocapturemodule::VideoCaptureImpl {

public:

    GeoVideoCaptureModule(int32_t id);

    virtual ~GeoVideoCaptureModule();

    /**
     * \brief Initialize mxuvc
     *
     * \param socket    /var/run/madmux/chX.sock
     *
     * \returns 0 on success, non-0 otherwise.
     */
    int32_t init(const char* socket);

private:

    static void _madmux_video_cb(uint8_t* buffer, uint32_t size, void* user_data);
    void _video_cb(uint8_t* buffer, uint32_t size);

    webrtc::VideoCaptureCapability _capability;

    webrtc::CriticalSectionWrapper* _capture_cs;

    bool _capturing;

    struct mdx_stream* _stream;

// webrtc::videocapturemodule::VideoCaptureImpl

public:

    static rtc::scoped_refptr<VideoCaptureModule> Create(const int32_t id,
            const char* device);

    static DeviceInfo* CreateDeviceInfo(const int32_t id);

    virtual int32_t StartCapture(const webrtc::VideoCaptureCapability& capability);

    virtual int32_t StopCapture();

    virtual bool CaptureStarted();

    virtual int32_t CaptureSettings(webrtc::VideoCaptureCapability& settings);

};

/**
 * \brief cricket::GeoVideoDeviceCapturerFactory specialization for GEO Semi
 */
class GeoVideoDeviceCapturerFactory : public cricket::WebRtcVideoDeviceCapturerFactory {

public:

    GeoVideoDeviceCapturerFactory(VideoCaptureModuleRegistryPtr module_reg);

private:

    VideoCaptureModuleRegistryPtr _module_reg;

// cricket::WebRtcVideoDeviceCapturerFactory

public:

    virtual cricket::VideoCapturer* Create(const cricket::Device& device);

};

#endif // USE_MADMUX

/**
 * \brief Represents a ROS topics as a WebRTC video capture device.
 */
class ROSVideoCaptureModule : public webrtc::videocapturemodule::VideoCaptureImpl {

public:

    ROSVideoCaptureModule(int32_t id);

    virtual ~ROSVideoCaptureModule();

    /**
     * \brief Subscribes to an image ROS topic.
     *
     * \param deviceUniqueIdUTF8 ROS topic name.
     * \returns 0 on success, non-0 otherwise.
     */
    int32_t init(const char* deviceUniqueIdUTF8);

private:

    class CaptureThread : public rtc::Thread {

    public:
        CaptureThread (ROSVideoCaptureModule &parent);

        ~CaptureThread();

    private:

        ROSVideoCaptureModule &_parent;

    // rtc::Thread

    public:

        virtual void Run();

    };

    void _image_callback(const sensor_msgs::ImageConstPtr& msg);

    webrtc::VideoCaptureCapability _capability;

    webrtc::CriticalSectionWrapper* _capture_cs;

    bool _capturing;

    ros::Subscriber _subscriber;

    ros::NodeHandle _nh;

    ros::CallbackQueue _image_q;

    CaptureThread* _capture_thd;

// webrtc::videocapturemodule::VideoCaptureImpl

public:

    static rtc::scoped_refptr<VideoCaptureModule> Create(const int32_t id, const char* deviceUniqueIdUTF8);

    static DeviceInfo* CreateDeviceInfo(const int32_t id);

    virtual int32_t StartCapture(const webrtc::VideoCaptureCapability& capability);

    virtual int32_t StopCapture();

    virtual bool CaptureStarted();

    virtual int32_t CaptureSettings(webrtc::VideoCaptureCapability& settings);

};

struct WebRTCVideoCaptureDeviceInfo {

    static void scan(std::vector<WebRTCVideoCaptureDeviceInfo> &infos);

    struct find_by_name {

        bool operator() (const WebRTCVideoCaptureDeviceInfo & o) {
            return o.name == name;
        }

        std::string name;

    };

    struct find_by_file_id {

        bool operator() (const WebRTCVideoCaptureDeviceInfo& o) {
            return o.file_id == file_id;
        }

        std::string file_id;

    };

    std::string name;
    std::string file_id;
    std::string unique_id;
    std::string product_id;

};
/**
 * \brief Collection of ROS topics to adapt as WebRTC video capture devices.
 */
class ROSVideoCaptureTopics {

public:

    /// Query master for all ROS topics that can be adapted.
    static ROSVideoCaptureTopics scan();

    ROSVideoCaptureTopics();

    ROSVideoCaptureTopics(const std::vector<ros::master::TopicInfo> &values);

    const ros::master::TopicInfo& get(size_t i) const { return _values[i]; }

    size_t size() const { return _values.size(); }

    void add(const std::string& name);

    int find(const std::string& name) const;

private:

    std::vector<ros::master::TopicInfo> _values;

};

typedef std::shared_ptr<ROSVideoCaptureTopics> ROSVideoCaptureTopicsPtr;

typedef std::shared_ptr<ROSVideoCaptureTopics const> ROSVideoCaptureTopicsConstPtr;


/**
 * \brief Represents available ROS topics as video capture devices to WebRTC.
 */
class ROSVideoCaptureDeviceInfo : public webrtc::videocapturemodule::DeviceInfoImpl {

public:

    ROSVideoCaptureDeviceInfo(const int32_t id);

    virtual ~ROSVideoCaptureDeviceInfo();

    bool init(ROSVideoCaptureTopicsConstPtr topics);

private:

    struct topic_name_less_than {

        inline bool operator() (const ros::master::TopicInfo& a, const ros::master::TopicInfo& b) {
           return (a.name < b.name);
       }

    };

    ROSVideoCaptureTopicsConstPtr _topics;

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

    virtual int32_t CreateCapabilityMap(const char* deviceUniqueIdUTF8);

    virtual int32_t DisplayCaptureSettingsDialogBox(
        const char* deviceUniqueIdUTF8,
        const char* dialogTitleUTF8,
        void* parentWindow,
        uint32_t positionX,
        uint32_t positionY
    );

    int32_t Init();

};

/**
 * \brief cricket::WebRtcVideoDeviceCapturerFactory specialization for ROS and tracking webrtc::VideoCaptureModule.
 */
class ROSVideoDeviceCapturerFactory : public cricket::WebRtcVideoDeviceCapturerFactory {

public:

    ROSVideoDeviceCapturerFactory(
        VideoCaptureModuleRegistryPtr module_reg,
        ROSVideoCaptureTopicsConstPtr topics);

private:

    ROSVideoCaptureTopicsConstPtr _topics;
    VideoCaptureModuleRegistryPtr _module_reg;

// cricket::WebRtcVideoDeviceCapturerFactory

public:

    virtual cricket::VideoCapturer* Create(const cricket::Device& device);

};

/**
 * \brief cricket::WebRtcVideoDeviceCapturerFactory specialization for tracking webrtc::VideoCaptureModule.
 */
class WebRTCVideoDeviceCapturerFactory : public cricket::WebRtcVideoDeviceCapturerFactory {

public:

    WebRTCVideoDeviceCapturerFactory(VideoCaptureModuleRegistryPtr module_reg);

private:

    VideoCaptureModuleRegistryPtr _module_reg;

// cricket::WebRtcVideoDeviceCapturerFactory

public:

    virtual cricket::VideoCapturer* Create(const cricket::Device& device);

};

#endif /* ROS_WEBRTC_VIDEO_CAPTURE_H_ */
