#include "video_capture.h"

#include <algorithm>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <webrtc/system_wrappers/interface/ref_count.h>

// VideoCaptureDeviceInfo

VideoCaptureDeviceInfo::VideoCaptureDeviceInfo(
    const int32_t id
    ) : DeviceInfoImpl(id) {
}

VideoCaptureDeviceInfo::~VideoCaptureDeviceInfo() {
}

uint32_t VideoCaptureDeviceInfo::NumberOfDevices() {
    // count all published topics w/ date-type sensor_msgs/Image
    uint32_t count = 0;
    ros::master::V_TopicInfo topics;
    if (!ros::master::getTopics(topics)) {
        ROS_WARN_STREAM("failed to get topics");
        return count;
    }
    std::sort(topics.begin(), topics.end(), topic_name_less_than());
    for (size_t i = 0; i < topics.size(); i++) {
        const ros::master::TopicInfo& topic = topics[i];
        if (topic.datatype == "sensor_msgs/Image") {
            count++;
        }
    }
    return count;
}

int32_t VideoCaptureDeviceInfo::GetDeviceName(
    uint32_t deviceNumber,
    char* deviceNameUTF8,
    uint32_t deviceNameLength,
    char* deviceUniqueIdUTF8,
    uint32_t deviceUniqueIdUTF8Length,
    char* productUniqueIdUTF8,
    uint32_t productUniqueIdUTF8Length
    ) {
    // find topic by index (brittle)
    ros::master::V_TopicInfo topics;
    if (!ros::master::getTopics(topics)) {
        ROS_WARN_STREAM("failed to get topics");
        return -1;
    }
    std::sort(topics.begin(), topics.end(), topic_name_less_than());
    size_t i = 0, number = 0, index = 0;
    for (; i < topics.size(); i++) {
        const ros::master::TopicInfo& topic = topics[i];
        if (topic.datatype == "sensor_msgs/Image") {
            if (number == deviceNumber)
                break;
            number++;
        }
    }
    if (number != deviceNumber) {
        return -1;
    }
    const ros::master::TopicInfo& topic = topics[i];

    // output topic name to id
    if (deviceNameLength >= topic.name.size() + 1) {
        memcpy(deviceNameUTF8, topic.name.c_str(), topic.name.size() + 1);
    } else {
        ROS_ERROR("buffer passed is too small");
        return -1;
    }

    // output topic name unique id
    if (deviceUniqueIdUTF8Length >= topic.name.size() + 1) {
        memcpy(deviceUniqueIdUTF8, topic.name.c_str(), topic.name.size() + 1);
    } else {
        ROS_ERROR("buffer passed is too small");
        return -1;
    }

    return 0;
}

int32_t VideoCaptureDeviceInfo::CreateCapabilityMap (const char* deviceUniqueIdUTF8) {
    // find topic by unique id
    ros::master::V_TopicInfo topics;
    if (!ros::master::getTopics(topics)) {
        ROS_WARN_STREAM("failed to get topics");
        return -1;
    }
    size_t index = -1;
    for (size_t i = 0; i < topics.size(); i++) {
        const ros::master::TopicInfo& topic = topics[i];
        if (topic.datatype == "sensor_msgs/Image" && topic.name == deviceUniqueIdUTF8) {
            index = i;
            break;
        }
    }
    if (index == -1) {
        ROS_ERROR_STREAM("no matching device  for '" << deviceUniqueIdUTF8 << "'found");
        return -1;
    }
    const ros::master::TopicInfo& topic = topics[index];

    webrtc::RawVideoType formats[] {
//        webrtc::kVideoI420,
//        webrtc::kVideoYV12,
//        webrtc::kVideoYUY2,
//        webrtc::kVideoUYVY,
//        webrtc::kVideoIYUV,
//        webrtc::kVideoARGB,
        webrtc::kVideoRGB24,
//        webrtc::kVideoRGB565,
//        webrtc::kVideoARGB4444,
//        webrtc::kVideoARGB1555,
//        webrtc::kVideoMJPEG,
//        webrtc::kVideoNV12,
//        webrtc::kVideoNV21,
//        webrtc::kVideoBGRA,
    };

    unsigned int sizes[][2] = {
        { 128, 96 },
        { 160, 120 },
        { 176, 144 },
        { 320, 240 },
        { 352, 288 },
        { 640, 480 },
        { 704, 576 },
        { 800, 600 },
        { 960, 720 },
        { 1280, 720 },
        { 1024, 768 },
        { 1440, 1080 },
        { 1920, 1080 }
    };

    for (size_t format_idx = 0; format_idx < sizeof(formats) / sizeof(formats[0]); format_idx++) {
        for (size_t size_idx = 0; size_idx < sizeof(sizes) / sizeof(sizes[0]); size_idx++) {
            webrtc::VideoCaptureCapability cap;

            cap.codecType = webrtc::kVideoCodecUnknown;
            cap.interlaced = false;
            cap.expectedCaptureDelay = 120; // TODO: what's this?
            cap.maxFPS = 30; // TODO: or 15?
            cap.rawType = formats[format_idx];
            cap.width = sizes[size_idx][0];
            cap.height = sizes[size_idx][1];

            _captureCapabilities.push_back(cap);
        }
    }

   // record topic whose capabilities are in _captureCapabilities
   _lastUsedDeviceNameLength = strlen(deviceUniqueIdUTF8);
   _lastUsedDeviceName = (char*) realloc(_lastUsedDeviceName, _lastUsedDeviceNameLength + 1);
   memcpy(_lastUsedDeviceName, deviceUniqueIdUTF8, _lastUsedDeviceNameLength + 1);

   ROS_INFO_STREAM(
        "loaded " << _captureCapabilities.size() << " capabilities for '" << deviceUniqueIdUTF8 << "'";
   );
   return _captureCapabilities.size();
}

int32_t VideoCaptureDeviceInfo::DisplayCaptureSettingsDialogBox(
    const char* deviceUniqueIdUTF8,
    const char* dialogTitleUTF8,
    void* parentWindow,
    uint32_t positionX,
    uint32_t positionY
    ) {
    return -1;  // not supported
}

int32_t VideoCaptureDeviceInfo::Init() {
    return 0;  // do nothing
}

// VideoCaptureModule

VideoCaptureModule::VideoCaptureModule(int32_t id) :
    VideoCaptureImpl(id),
    _capture_cs(webrtc::CriticalSectionWrapper::CreateCriticalSection()),
    _capturing(false),
    _capture_thd(NULL) {
    _nh.setCallbackQueue(&_cb_q);
}

VideoCaptureModule::~VideoCaptureModule() {
    StopCapture();
    _subscriber.shutdown();
    _nh.setCallbackQueue(NULL);
    if (_capture_cs) {
        delete _capture_cs;
        _capture_cs = NULL;
    }
}

int32_t VideoCaptureModule::init(const char* deviceUniqueIdUTF8) {
    // find topic by unique id
    ros::master::V_TopicInfo topics;
    if (!ros::master::getTopics(topics)) {
        ROS_WARN_STREAM("failed to get topics");
        return -1;
    }
    size_t index = -1;
    for (size_t i = 0; i < topics.size(); i++) {
        const ros::master::TopicInfo& topic = topics[i];
        if (topic.datatype == "sensor_msgs/Image" && topic.name == deviceUniqueIdUTF8) {
            index = i;
            break;
        }
    }
    if (index == -1) {
        ROS_ERROR("no matching device  for '%s' found", deviceUniqueIdUTF8);
        return -1;
    }
    _topic = topics[index].name;

    // subscribe to the topic
    _subscriber = _nh.subscribe(_topic, 100, &VideoCaptureModule::_enqueue_image, this);

    return 0;
}

bool VideoCaptureModule::_capture(void* obj) {
    return static_cast<VideoCaptureModule*>(obj)->_capture_image();
}

bool VideoCaptureModule::_capture_image() {
    ros::WallDuration timeout(1.0);

    {
        // lock
        webrtc::CriticalSectionScoped cs(_capture_cs);
        if (!_capturing) {
            return false;
        }

        // queue image
        ros::CallbackQueue::CallOneResult result = ros::CallbackQueue::TryAgain;
        while (result == ros::CallbackQueue::TryAgain) {
            result = _cb_q.callOne();
        }
        if (!_capturing) {
            return false;
        }
        if (result != ros::CallbackQueue::Called) {
            return true;
        }

        // deliver
        Frame frame = _frames.front();
        _frames.pop();
        // TODO: check return code?
        IncomingFrame(&frame.buffer[0], frame.buffer.size(), _capability);
    }

    usleep(0);  // yield
    return true;
}

void VideoCaptureModule::_enqueue_image(const sensor_msgs::ImageConstPtr& image) {
    Frame frame;

    // encode
    cv_bridge::CvImagePtr cv_msg = cv_bridge::toCvCopy(image, _frame_encoding);
    if (cv_msg == NULL) {
        ROS_ERROR(
            "failed to copy sensor image '%s' to cv image '%s'",
            image->encoding.c_str(), _frame_encoding.c_str()
        );
        return;
    }
    cv::Mat cv_image;
    cv::swap(cv_msg->image, cv_image);

    // resize
    if (image->width != _capability.width || image->height != _capability.height) {
        cv::Mat cv_resize_image;
        cv::resize(cv_image, cv_resize_image, cv::Size(_capability.width, _capability.height));
        cv::swap(cv_image, cv_resize_image);
    }

    frame.buffer.resize(cv_image.rows * cv_image.cols * 3);
    uint8_t* b = &frame.buffer[0];
    cv::Vec3b p;
    for (int row = 0; row != cv_image.rows; row += 1) {
        for (int col = 0; col != cv_image.cols; col += 1) {
            p = cv_image.at<cv::Vec3b>(row, col);
            *(b++) = p[2];
            *(b++) = p[1];
            *(b++) = p[0];
        }
    }
    _frames.push(frame);
}

webrtc::VideoCaptureModule* VideoCaptureModule::Create(const int32_t id, const char* deviceUniqueIdUTF8) {
    webrtc::RefCountImpl<VideoCaptureModule>* obj = new webrtc::RefCountImpl<VideoCaptureModule>(id);
    if (!obj || obj->init(deviceUniqueIdUTF8) != 0) {
        delete obj;
        obj = NULL;
    }
    return obj;
}

VideoCaptureModule::DeviceInfo* VideoCaptureModule::CreateDeviceInfo(const int32_t id) {
    return new VideoCaptureDeviceInfo(id);
}

int32_t VideoCaptureModule::StartCapture(const webrtc::VideoCaptureCapability& capability) {
    if (_capturing) {
        if (capability.width == _capability.width &&
            capability.height == _capability.height &&
            capability.rawType == _capability.rawType) {
            // already started w/ same profile
            return 0;
        } else {
            // profile changes, so stop
            StopCapture();
        }
    }

    webrtc::CriticalSectionScoped cs(_capture_cs);

    // sensor_msgs::frame_encodings for capability
    std::string frame_encoding;
    switch (capability.rawType) {
        case webrtc::kVideoUYVY:
            frame_encoding = sensor_msgs::image_encodings::YUV422;
            break;
        case webrtc::kVideoRGB24:
            frame_encoding = sensor_msgs::image_encodings::RGB8;
            break;
        default:
            return -1;
    }

    // frames
    FrameQueue frames;

    //start capture thread;
    if (_capture_thd == NULL) {
        _capture_thd = webrtc::ThreadWrapper::CreateThread(
            VideoCaptureModule::_capture, this, webrtc::kHighPriority
        );
        if (_capture_thd == NULL) {
            return -1;
        }
        unsigned int id;
        _capture_thd->Start(id);
    }

    // done
    _frame_encoding = frame_encoding;
    std::swap(_frames, _frames);
    _capability = capability;
    _capturing = true;

    return 0;
}

int32_t VideoCaptureModule::StopCapture() {
    if (_capture_thd != NULL) {
        if (_capture_thd->Stop()) {
            delete _capture_thd;
            _capture_thd = NULL;
        } else {
            ROS_ERROR_STREAM("could not stop capture thread, leaking it ...");
            assert(false);
            _capture_thd = NULL;
        }
    }

    {
        webrtc::CriticalSectionScoped cs(_capture_cs);
        if (_capturing) {
            _capturing = false;
            FrameQueue empty;
            std::swap(empty, _frames);
            _cb_q.clear();
        }
    }

    return 0;
}

bool VideoCaptureModule::CaptureStarted() {
    return _capturing;
}

int32_t VideoCaptureModule::CaptureSettings(webrtc::VideoCaptureCapability& settings) {
    settings = _capability;
    return 0;
}


// WebRtcVcmFactory

WebRtcVcmFactory::~WebRtcVcmFactory() {
}

webrtc::VideoCaptureModule* WebRtcVcmFactory::Create(int id, const char* device) {
    return VideoCaptureModule::Create(id, device);
}

webrtc::VideoCaptureModule::DeviceInfo* WebRtcVcmFactory::CreateDeviceInfo(int id) {
    return VideoCaptureModule::CreateDeviceInfo(id);
}

void WebRtcVcmFactory::DestroyDeviceInfo(webrtc::VideoCaptureModule::DeviceInfo* info) {
    // do nothing
}
