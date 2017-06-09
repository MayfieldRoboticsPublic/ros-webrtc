#include "video_capture.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.hpp>
#include <sensor_msgs/Image.h>
#include <webrtc/media/engine/webrtcvideocapturer.h>
#include <webrtc/modules/video_capture/video_capture_factory.h>

// VideoCaptureModuleRegistry

void VideoCaptureModuleRegistry::add(cricket::VideoCapturer *capturer, webrtc::VideoCaptureModule *module) {
    add(capturer->GetId(), module);
}

void VideoCaptureModuleRegistry::add(const std::string &name, webrtc::VideoCaptureModule *module) {
    assert(module != NULL);
    ROS_INFO_STREAM("registering vcm w/ name '" << name << "'");
    _modules[name] = module;
}

bool VideoCaptureModuleRegistry::remove(cricket::VideoCapturer *capturer) {
    return remove(capturer->GetId());
}

bool VideoCaptureModuleRegistry::remove(const std::string &name) {
    auto i = _modules.find(name);
    if (i == _modules.end()) {
        return false;
    }
    _modules.erase(i);
    return true;
}

void VideoCaptureModuleRegistry::remove_all() {
    _modules.clear();
}

webrtc::VideoCaptureModule* VideoCaptureModuleRegistry::find(cricket::VideoCapturer *capturer) const {
    return find(capturer->GetId());
}

webrtc::VideoCaptureModule* VideoCaptureModuleRegistry::find(const std::string &name) const {
    auto i = _modules.find(name);
    if (i == _modules.end()) {
        return NULL;
    }
    webrtc::VideoCaptureModule* module = (*i).second.get();
    module->AddRef();
    return module;
}

std::vector<std::string> VideoCaptureModuleRegistry::ids() const {
    std::vector<std::string> ids;
    for(auto i = _modules.begin(); i != _modules.end(); i++) {
      ids.push_back(i->first);
    }
    return ids;
}

// ROSVideoCaptureModule

ROSVideoCaptureModule::ROSVideoCaptureModule(int32_t id) :
    VideoCaptureImpl(id),
    _capture_cs(webrtc::CriticalSectionWrapper::CreateCriticalSection()),
    _capturing(false),
    _capture_thd(NULL) {
    _nh.setCallbackQueue(&_image_q);
}

ROSVideoCaptureModule::~ROSVideoCaptureModule() {
    StopCapture();
    _subscriber.shutdown();
    _nh.setCallbackQueue(NULL);
    if (_capture_cs) {
        delete _capture_cs;
        _capture_cs = NULL;
    }
}

int32_t ROSVideoCaptureModule::init(const char* deviceUniqueIdUTF8) {
    try {
        _subscriber = _nh.subscribe(
            deviceUniqueIdUTF8,
            1,
            &ROSVideoCaptureModule::_image_callback,
            this
        );
    }
    catch (ros::Exception& ex) {
        ROS_ERROR(
            "subscribe to video capture device topic '%s' failed - %s",
            deviceUniqueIdUTF8, ex.what()
        );
        return -1;
    }
    return 0;
}

void ROSVideoCaptureModule::_image_callback(const sensor_msgs::ImageConstPtr& msg) {
    // force to bgr8
    cv::Mat bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
    if (bgr.cols != _capability.width || bgr.rows != _capability.height) {
        cv::resize(bgr, bgr, cv::Size(_capability.width, _capability.height));
    }

    // convert to I420
    cv::Mat yuv(bgr.rows, bgr.cols, CV_8UC4);
    cv::cvtColor(bgr, yuv, CV_BGR2YUV_I420);

    // adjust caps
    _capability.width = bgr.cols;
    _capability.height = bgr.rows;
    _capability.rawType = webrtc::kVideoI420;

    // send it along
    IncomingFrame(yuv.data, yuv.rows * yuv.step, _capability, msg->header.stamp.toNSec());
}

rtc::scoped_refptr<webrtc::VideoCaptureModule> ROSVideoCaptureModule::Create(const int32_t id, const char* deviceUniqueIdUTF8) {
    rtc::scoped_refptr<ROSVideoCaptureModule> obj = new rtc::RefCountedObject<ROSVideoCaptureModule>(id);
    if (!obj || obj->init(deviceUniqueIdUTF8) != 0) {
        obj = NULL;
    }
    return obj;
}

ROSVideoCaptureModule::DeviceInfo* ROSVideoCaptureModule::CreateDeviceInfo(const int32_t id) {
    return new ROSVideoCaptureDeviceInfo(id);
}

int32_t ROSVideoCaptureModule::StartCapture(const webrtc::VideoCaptureCapability& capability) {
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

    // start capture thread
    if (_capture_thd == NULL) {
        _capture_thd = new CaptureThread(*this);
        if (_capture_thd == NULL) {
            return -1;
        }
        _capture_thd->Start();
    }

    // done
    _capability = capability;
    _capturing = true;

    return 0;
}

int32_t ROSVideoCaptureModule::StopCapture() {
    if (_capture_thd != NULL) {
        ROS_INFO("stopping capture thread ...");
        _capture_thd->Stop();
        ROS_INFO("stopped capture thread");
        delete _capture_thd;
        _capture_thd = NULL;
    }

    {
        webrtc::CriticalSectionScoped cs(_capture_cs);
        if (_capturing) {
            _capturing = false;
            _image_q.clear();
        }
    }

    return 0;
}

bool ROSVideoCaptureModule::CaptureStarted() {
    return _capturing;
}

int32_t ROSVideoCaptureModule::CaptureSettings(webrtc::VideoCaptureCapability& settings) {
    settings = _capability;
    return 0;
}

// ROSVideoCaptureModule::CaptureThread

ROSVideoCaptureModule::CaptureThread::CaptureThread(
    ROSVideoCaptureModule &parent) : _parent(parent) {
}

ROSVideoCaptureModule::CaptureThread::~CaptureThread() {
    Stop();
}
void ROSVideoCaptureModule::CaptureThread::Run() {
    ros::CallbackQueue::CallOneResult result = ros::CallbackQueue::TryAgain;

    while(!fStop_) {
        {
            // lock
            webrtc::CriticalSectionScoped cs(_parent._capture_cs);
            if (fStop_)
                break;

            // poll
            do {
                // NOTE: handler is ROSVideoCaptureModule::_image_callback
                result = _parent._image_q.callOne();
            } while (result == ros::CallbackQueue::TryAgain && !fStop_);
        }

        if (!fStop_ && result == ros::CallbackQueue::Called) {
            if (!ProcessMessages(0)) {
                break;
            }
        } else {
            usleep(0);  // yield
        }
    }
}

#ifdef USE_MADMUX
// GeoVideoCaptureModule

GeoVideoCaptureModule::GeoVideoCaptureModule(int32_t id)
    : VideoCaptureImpl(id)
    , _capture_cs(webrtc::CriticalSectionWrapper::CreateCriticalSection())
    , _capturing(false)
{ }

GeoVideoCaptureModule::~GeoVideoCaptureModule()
{
    StopCapture();
    if (_capture_cs) {
        delete _capture_cs;
        _capture_cs = NULL;
    }
}

int32_t GeoVideoCaptureModule::init(const char* device)
{
    _stream = mdx_open(GEO_CAM_SOCK);

    return 0;
}

void GeoVideoCaptureModule::_madmux_video_cb(uint8_t* buffer, uint32_t size,
        void* user_data)
{
    GeoVideoCaptureModule* mod = static_cast<GeoVideoCaptureModule*>(user_data);
    mod->_video_cb(buffer, size);
}

void GeoVideoCaptureModule::_video_cb(uint8_t* buffer, uint32_t size)
{
    webrtc::VideoCaptureCapability frameInfo;
    frameInfo.width = GEO_CAM_WIDTH;
    frameInfo.height = GEO_CAM_HEIGHT;
    frameInfo.rawType = webrtc::kVideoMJPEG;

    // convert to to I420 if needed
    IncomingFrame(buffer, size, frameInfo);
}

rtc::scoped_refptr<webrtc::VideoCaptureModule> GeoVideoCaptureModule::Create(
        const int32_t id, const char* device)
{
    rtc::scoped_refptr<GeoVideoCaptureModule> obj =
        new rtc::RefCountedObject<GeoVideoCaptureModule>(id);
    if (!obj || obj->init(device) != 0) {
        obj = NULL;
    }
    return obj;
}

int32_t GeoVideoCaptureModule::StartCapture(const webrtc::VideoCaptureCapability& capability) {
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

    // start capture thread
    mdx_register_cb(_stream, &GeoVideoCaptureModule::_madmux_video_cb, this);

    // done
    _capability = capability;
    _capturing = true;

    return 0;
}

int32_t GeoVideoCaptureModule::StopCapture() {
    webrtc::CriticalSectionScoped cs(_capture_cs);

    if (_capturing) {
        _capturing = false;
        mdx_register_cb(_stream, nullptr, nullptr);
    }

    return 0;
}

bool GeoVideoCaptureModule::CaptureStarted() {
    return _capturing;
}

int32_t GeoVideoCaptureModule::CaptureSettings(webrtc::VideoCaptureCapability& settings) {
    settings = _capability;
    return 0;
}

// GeoVcmFactory

/**
 * \brief cricket::WebRtcVcmFactoryInterface implementation for tracking GeoVideoCaptureModule
 */
class GeoVcmFactory : public cricket::WebRtcVcmFactoryInterface {

public:

    GeoVcmFactory( const cricket::Device& device,
            VideoCaptureModuleRegistryPtr module_reg)
        : _device_id(device.id)
        , _module_reg(module_reg)
    { }

private:

    std::string _device_id;
    VideoCaptureModuleRegistryPtr _module_reg;

// cricket::WebRtcVcmFactoryInterface

public:

    virtual rtc::scoped_refptr<webrtc::VideoCaptureModule> Create(int id, const char* device) {
        rtc::scoped_refptr<webrtc::VideoCaptureModule> module(
            GeoVideoCaptureModule::Create(id, device)
        );
        if (_module_reg != NULL) {
            _module_reg->add(_device_id, module.get());
        }
        return module;
    }

    virtual webrtc::VideoCaptureModule::DeviceInfo* CreateDeviceInfo(int id) {
        return webrtc::VideoCaptureFactory::CreateDeviceInfo(id);
    }

    virtual void DestroyDeviceInfo(webrtc::VideoCaptureModule::DeviceInfo* info) {
        if (info) {
            delete info;
        }
    }

};

// GeoVideoDeviceCapturerFactor

GeoVideoDeviceCapturerFactory::GeoVideoDeviceCapturerFactory (
    VideoCaptureModuleRegistryPtr module_reg)
    : _module_reg(module_reg)
{ }

cricket::VideoCapturer* GeoVideoDeviceCapturerFactory::Create(const cricket::Device& device) {
    std::unique_ptr<cricket::WebRtcVideoCapturer> capturer(
        new cricket::WebRtcVideoCapturer(
            new GeoVcmFactory(device, _module_reg)
        )
    );
    if (capturer->Init(device) < 0) {
        return NULL;
    }
    return capturer.release();
}
#endif // USE_MADMUX

// WebRTCVideoCaptureDevices

void WebRTCVideoCaptureDeviceInfo::scan(std::vector<WebRTCVideoCaptureDeviceInfo> &infos) {
    std::unique_ptr<webrtc::VideoCaptureModule::DeviceInfo> dev_info(
        webrtc::VideoCaptureFactory::CreateDeviceInfo(0)
    );
    auto nb_device = dev_info->NumberOfDevices();
    for (auto i = 0; i < nb_device; i += 1) {
        char file_id[1024] = {0};
        char name[1024] = {0};
        char unique_id[1024] = {0};
        char product_id[1024] = {0};
        auto rc = dev_info->GetDeviceName(
            i,
            name, sizeof(name),
            unique_id, sizeof(unique_id),
            product_id, sizeof(product_id)
        );
        if (rc != 0) {
            continue;
        }
        sprintf(file_id, "/dev/video%d", i);
        infos.push_back({
            name,
            file_id,
            unique_id,
            product_id
        });
    }
}

// ROSVideoCaptureTopics

ROSVideoCaptureTopics ROSVideoCaptureTopics::scan() {
    std::vector<ros::master::TopicInfo> topics;
    ros::master::V_TopicInfo candidates;
    if (!ros::master::getTopics(candidates)) {
        throw std::runtime_error("failed to get topics");
    }
    for (size_t i = 0; i < candidates.size(); i++) {
        const ros::master::TopicInfo& topic = candidates[i];
        if (topic.datatype != "sensor_msgs/Image")
            continue;
        ROS_INFO(
            "found ros video capture device - topic '%s', data type '%s'",
            topic.name.c_str(), topic.datatype.c_str()
        );
        topics.push_back(topic);
    }
    return ROSVideoCaptureTopics(topics);
}

ROSVideoCaptureTopics::ROSVideoCaptureTopics() {
}

ROSVideoCaptureTopics::ROSVideoCaptureTopics(
    const std::vector<ros::master::TopicInfo> &values
    ) : _values(values) {
}

void ROSVideoCaptureTopics::add(const std::string& name) {
    _values.push_back(ros::master::TopicInfo(name, "sensor_msgs/Image"));
}

int ROSVideoCaptureTopics::find(const std::string& name) const {
    for (size_t i = 0; i != _values.size(); i++) {
        if (_values[i].name == name) {
            return i;
        }
    }
    return -1;
}

// ROSVideoCaptureDeviceInfo

ROSVideoCaptureDeviceInfo::ROSVideoCaptureDeviceInfo(const int32_t id) : DeviceInfoImpl(id) {
}

ROSVideoCaptureDeviceInfo::~ROSVideoCaptureDeviceInfo() {
}

bool ROSVideoCaptureDeviceInfo::init(ROSVideoCaptureTopicsConstPtr topics) {
    _topics = topics;
    return true;
}

uint32_t ROSVideoCaptureDeviceInfo::NumberOfDevices() {
    return _topics->size();
}

int32_t ROSVideoCaptureDeviceInfo::GetDeviceName(
    uint32_t deviceNumber,
    char* deviceNameUTF8,
    uint32_t deviceNameLength,
    char* deviceUniqueIdUTF8,
    uint32_t deviceUniqueIdUTF8Length,
    char* productUniqueIdUTF8,
    uint32_t productUniqueIdUTF8Length
    ) {
    // find topic by index
    if (_topics->size() <= deviceNumber) {
        return -1;
    }
    const ros::master::TopicInfo& topic = _topics->get(deviceNumber);

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

int32_t ROSVideoCaptureDeviceInfo::CreateCapabilityMap(const char* deviceUniqueIdUTF8) {
    // find topic by unique id
    int i = _topics->find(deviceUniqueIdUTF8);
    if (i == -1)
        return -1;
    const ros::master::TopicInfo& topic = _topics->get(i);

    webrtc::RawVideoType formats[] {
        webrtc::kVideoRGB24,
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

int32_t ROSVideoCaptureDeviceInfo::DisplayCaptureSettingsDialogBox(
    const char* deviceUniqueIdUTF8,
    const char* dialogTitleUTF8,
    void* parentWindow,
    uint32_t positionX,
    uint32_t positionY
    ) {
    return -1;  // not supported
}

int32_t ROSVideoCaptureDeviceInfo::Init() {
    return 0;  // do nothing
}

// ROSWebRtcVcmFactory

/**
 * \brief Factory for exposing ROS video capture module and devices to cricket::WebRtcVideoCapturer.
 */
class ROSWebRtcVcmFactory : public cricket::WebRtcVcmFactoryInterface {

public:

    ROSWebRtcVcmFactory(
        const cricket::Device &device,
        ROSVideoCaptureTopicsConstPtr topics,
        VideoCaptureModuleRegistryPtr module_reg) :
            _device_id(device.id),
            _topics(topics),
            _module_reg(module_reg) {
    }

    virtual ~ROSWebRtcVcmFactory() {}

private:

    std::string _device_id;
    ROSVideoCaptureTopicsConstPtr _topics;
    VideoCaptureModuleRegistryPtr _module_reg;

// cricket::WebRtcVcmFactoryInterface

public:

    virtual rtc::scoped_refptr<webrtc::VideoCaptureModule> Create(int id, const char* device) {
        rtc::scoped_refptr<webrtc::VideoCaptureModule> module(
            ROSVideoCaptureModule::Create(id, device)
        );
        if (_module_reg != NULL) {
            _module_reg->add(_device_id, module.get());
        }
        return module;
    }

    virtual webrtc::VideoCaptureModule::DeviceInfo* CreateDeviceInfo(int id) {
        std::unique_ptr<webrtc::VideoCaptureModule::DeviceInfo> devices(
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

// WebRtcVcmFactory

/**
 * \brief cricket::WebRtcVcmFactoryInterface implementation for tracking webrtc::VideoCaptureModule.
 */
class WebRtcVcmFactory : public cricket::WebRtcVcmFactoryInterface {

public:

    WebRtcVcmFactory(
        const cricket::Device& device,
        VideoCaptureModuleRegistryPtr module_reg) :
        _device_id(device.id),
        _module_reg(module_reg) {
    }

private:

    std::string _device_id;
    VideoCaptureModuleRegistryPtr _module_reg;

// cricket::WebRtcVcmFactoryInterface

public:

    virtual rtc::scoped_refptr<webrtc::VideoCaptureModule> Create(int id, const char* device) {
        rtc::scoped_refptr<webrtc::VideoCaptureModule> module(
            webrtc::VideoCaptureFactory::Create(id, device)
        );
        if (_module_reg != NULL) {
            _module_reg->add(_device_id, module.get());
        }
        return module;
    }

    virtual webrtc::VideoCaptureModule::DeviceInfo* CreateDeviceInfo(int id) {
        return webrtc::VideoCaptureFactory::CreateDeviceInfo(id);
    }

    virtual void DestroyDeviceInfo(webrtc::VideoCaptureModule::DeviceInfo* info) {
        delete info;
    }

};

// ROSVideoDeviceCapturerFactor

ROSVideoDeviceCapturerFactory::ROSVideoDeviceCapturerFactory (
    VideoCaptureModuleRegistryPtr module_reg,
    ROSVideoCaptureTopicsConstPtr topics) :
        _topics(topics),
        _module_reg(module_reg) {
}

cricket::VideoCapturer* ROSVideoDeviceCapturerFactory::Create(const cricket::Device& device) {
    std::unique_ptr<cricket::WebRtcVideoCapturer> capturer(
        new cricket::WebRtcVideoCapturer(
            new ROSWebRtcVcmFactory(device, _topics, _module_reg)
        )
    );
    if (!capturer->Init(device)) {
        return NULL;
    }
    return capturer.release();
}

// WebRTCVideoDeviceCapturerFactory

WebRTCVideoDeviceCapturerFactory::WebRTCVideoDeviceCapturerFactory (
    VideoCaptureModuleRegistryPtr module_reg) :
        _module_reg(module_reg) {
};

cricket::VideoCapturer* WebRTCVideoDeviceCapturerFactory::Create(const cricket::Device& device) {
    std::unique_ptr<cricket::WebRtcVideoCapturer> capturer(
        new cricket::WebRtcVideoCapturer(new WebRtcVcmFactory(
            device, _module_reg
        ))
    );
    if (!capturer->Init(device)) {
        return NULL;
    }
    return capturer.release();
}
