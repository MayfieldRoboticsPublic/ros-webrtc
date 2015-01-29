#include "renderer.h"

#include <sensor_msgs/image_encodings.h>
#include <talk/media/base/videocommon.h>
#include <talk/media/base/videoframe.h>

// AudioSink

AudioSink::AudioSink(
    ros::NodeHandle& nh,
    const std::string& topic,
    webrtc::AudioTrackInterface* audio_track
    ) :
    _audio_track(audio_track),
    _rpub(nh.advertise<ros_webrtc::Audio>(topic, 1000)) {
    ROS_DEBUG_STREAM("registering audio renderer");
    _audio_track->AddSink(this);
}

AudioSink::~AudioSink() {
    ROS_DEBUG_STREAM("unregistering audio renderer");
    _audio_track->RemoveSink(this);
    _msg.header.stamp = ros::Time::now();
    _msg.header.seq += 1;
}

webrtc::AudioTrackInterface* AudioSink::audio_track() {
    return _audio_track;
}

void AudioSink::OnData(
        const void* audio_data,
        int bits_per_sample,
        int sample_rate,
        int number_of_channels,
        int number_of_frames
        ) {
    ROS_INFO_STREAM(
        "audio data -"
        << " bps : " << bits_per_sample
        <<" sample_rate: " << sample_rate
        << " number_of_channels: " << number_of_channels
        << " number_of_frames: " << number_of_frames
    );
}

// VideoRenderer

VideoRenderer::VideoRenderer(
    ros::NodeHandle nh,
    const std::string& topic,
    webrtc::VideoTrackInterface* video_track
    ) :
    _video_track(video_track),
    _rpub(nh.advertise<sensor_msgs::Image>(topic, 1000)) {
    ROS_DEBUG_STREAM("registering video renderer");
    _video_track->AddRenderer(this);
    _msg.encoding = sensor_msgs::image_encodings::BGR8;
    _msg.is_bigendian = false;
}

VideoRenderer::~VideoRenderer() {
    ROS_DEBUG_STREAM("unregistering video renderer");
    _video_track->RemoveRenderer(this);
}

webrtc::VideoTrackInterface* VideoRenderer::video_track() {
    return _video_track.get();
}

void VideoRenderer::SetSize(int width, int height) {
    ROS_DEBUG_STREAM("video size - width: " << width << " height: " << height);
    _msg.height = height;
    _msg.width = width;
    _msg.step = width * 3;
    _msg.data.resize(_msg.step * _msg.height);
}

void VideoRenderer::RenderFrame(const cricket::VideoFrame* frame) {
    ROS_DEBUG_STREAM("video render frame");
    _msg.header.stamp = ros::Time::now();
    _msg.header.seq += 1;
    frame->ConvertToRgbBuffer(
        cricket::FOURCC_BGR3,
        &_msg.data[0],
        _msg.data.size(),
        _msg.step
    );
    _rpub.publish(_msg);
}

// DataObserver

DataObserver::DataObserver(
    ros::Publisher& rpub,
    webrtc::DataChannelInterface* data_channel
    ) :
    _data_channel(data_channel),
    _rpub(rpub) {
    ROS_INFO_STREAM("registering data renderer for '" << _data_channel->label() << "' to '" << _rpub.getTopic() << "'");
    _data_channel->RegisterObserver(this);
}

DataObserver::~DataObserver() {
    ROS_INFO_STREAM("registering data renderer for '" << _data_channel->label() << "'");
    _data_channel->UnregisterObserver();
}

void DataObserver::OnStateChange() {
    ROS_INFO_STREAM("data state change for '" << _data_channel->label() << "' to '" << _data_channel->state() << "'");
}

void DataObserver::OnMessage(const webrtc::DataBuffer& buffer) {
    ROS_INFO_STREAM(
        "data message for '" << _data_channel->label() << "' - "
        << "binary=" << buffer.binary << ", "
        << "size=" << buffer.data.length()
    );
    ros_webrtc::Data msg;
    msg.encoding = buffer.binary ? "utf-8" : "binary";
    msg.buffer.insert(
        msg.buffer.end(),
        buffer.data.data(),
        buffer.data.data() + buffer.data.length()
    );
    _rpub.publish(msg);
}
