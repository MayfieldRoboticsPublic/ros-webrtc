#ifndef WEBRTC_RENDERER_H_
#define WEBRTC_RENDERER_H_

#include <string>

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <ros_webrtc/Audio.h>
#include <ros_webrtc/Data.h>
#include <sensor_msgs/Image.h>
#include <talk/app/webrtc/mediastreaminterface.h>
#include <talk/app/webrtc/datachannelinterface.h>
#include <webrtc/base/scoped_ref_ptr.h>

class AudioSink : public webrtc::AudioTrackSinkInterface {

public:
    AudioSink(
        ros::NodeHandle& nh,
        const std::string& topic,
        webrtc::AudioTrackInterface* audio_track
    );

    ~AudioSink();

    webrtc::AudioTrackInterface* audio_track();

private:
    rtc::scoped_refptr<webrtc::AudioTrackInterface> _audio_track;

    ros_webrtc::Audio _msg;

    ros::Publisher _rpub;

// webrtc::AudioTrackSinkInterface

public:

    virtual void OnData(
        const void* audio_data,
        int bits_per_sample,
        int sample_rate,
        int number_of_channels,
        int number_of_frames
    );

};

typedef boost::shared_ptr<AudioSink> AudioSinkPtr;

class VideoRenderer : public webrtc::VideoRendererInterface {

public:

    VideoRenderer(
        ros::NodeHandle nh,
        const std::string& topic,
        webrtc::VideoTrackInterface* video_track
    );

    ~VideoRenderer();

    webrtc::VideoTrackInterface* video_track();

private:

    rtc::scoped_refptr<webrtc::VideoTrackInterface> _video_track;

    ros::Publisher _rpub;

    sensor_msgs::Image _msg;

// webrtc::VideoRendererInterface

public:

    virtual void SetSize(int width, int height);

    virtual void RenderFrame(const cricket::VideoFrame* frame);

};

typedef boost::shared_ptr<VideoRenderer> VideoRendererPtr;

class DataObserver : public webrtc::DataChannelObserver {

public:

    DataObserver(
        ros::Publisher& rpub,
        webrtc::DataChannelInterface* data_channel
    );

    virtual ~DataObserver();

private:

    rtc::scoped_refptr<webrtc::DataChannelInterface> _data_channel;

    ros::Publisher _rpub;

// webrtc::DataChannelObserver

public:

    virtual void OnStateChange();

    virtual void OnMessage(const webrtc::DataBuffer& buffer);

};

typedef boost::shared_ptr<DataObserver> DataObserverPtr;

#endif /* WEBRTC_RENDERER_H_ */
