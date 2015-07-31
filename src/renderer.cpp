#include "renderer.h"

#include <json/json.h>
#include <sensor_msgs/image_encodings.h>
#include <talk/media/base/videocommon.h>
#include <talk/media/base/videoframe.h>

// AudioSink

AudioSink::AudioSink(
    ros::NodeHandle& nh,
    const std::string& topic,
    uint32_t queue_size,
    webrtc::AudioTrackInterface* audio_track
    ) :
    _audio_track(audio_track),
    _rpub(nh.advertise<ros_webrtc::Audio>(topic, queue_size)) {
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
    uint32_t queue_size,
    webrtc::VideoTrackInterface* video_track
    ) :
    _video_track(video_track),
    _rpub(nh.advertise<sensor_msgs::Image>(topic, queue_size)) {
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
    ros::NodeHandle& nh,
    const std::string& topic,
    uint32_t queue_size,
    webrtc::DataChannelInterface* data_channel
    ) :
    _dc(data_channel),
    _rpub(nh.advertise<ros_webrtc::Data>(topic, queue_size)) {
    ROS_INFO(
        "registering data renderer for '%s' to '%s'",
        _dc->label().c_str(), _rpub.getTopic().c_str()
    );
    _dc->RegisterObserver(this);
}

DataObserver::~DataObserver() {
    ROS_INFO("unregistering data renderer for '%s'", _dc->label().c_str());
    _dc->UnregisterObserver();
}

void DataObserver::OnStateChange() {
    ROS_INFO(
        "data state change for '%s' to '%d'",
        _dc->label().c_str(), _dc->state()
    );
}

// UnchunkedDataObserver

UnchunkedDataObserver::UnchunkedDataObserver(
    ros::NodeHandle& nh,
    const std::string& topic,
    uint32_t queue_size,
    webrtc::DataChannelInterface* data_channel
    ) : DataObserver(nh, topic, queue_size, data_channel) {
}

size_t UnchunkedDataObserver::reap() {
    return 0;
}

void UnchunkedDataObserver::OnMessage(const webrtc::DataBuffer& buffer) {
    ROS_INFO(
        "data message for '%s' - binary=%s, size=%zu",
        _dc->label().c_str(), buffer.binary ? "true" : "false", buffer.data.size()
    );
    ros_webrtc::Data msg;
    msg.label = _dc->label();
    msg.encoding = buffer.binary ? "utf-8" : "binary";
    msg.buffer.insert(
        msg.buffer.end(),
        buffer.data.data(),
        buffer.data.data() + buffer.data.size()
    );
    _rpub.publish(msg);
}

// ChunkedDataObserver

ChunkedDataObserver::ChunkedDataObserver(
    ros::NodeHandle& nh,
    const std::string& topic,
    uint32_t queue_size,
    webrtc::DataChannelInterface* data_channel
    ) : DataObserver(nh, topic, queue_size, data_channel) {
}

size_t ChunkedDataObserver::reap() {
    size_t count = 0;
    Messages::iterator i = _messages.begin();
    while (i != _messages.end()) {
        if ((*i).second->is_expired()) {
            ROS_WARN_STREAM(
                "data message for '" << _dc->label()
                << "' w/" << " id "  << (*i).second->id << " expired @" << (*i).second->expires_at
                << ", discarding ... "
            );
            count++;
            _messages.erase(i++);  // http://stackoverflow.com/a/596180
        } else {
            i++;
        }
    }
    return count;
}

void ChunkedDataObserver::OnMessage(const webrtc::DataBuffer& buffer) {
    ROS_INFO_STREAM(
        "data message for '" << _dc->label() << "' - "
        << "binary=" << buffer.binary << ", "
        << "size=" << buffer.data.size()
    );

    // deserialize chunk
    Json::Value chunk;
    Json::Reader reader;
    bool parsingSuccessful = reader.parse(
        buffer.data.data(),
        buffer.data.data() + buffer.data.size(),
        chunk
    );
    if (!parsingSuccessful) {
        ROS_WARN_STREAM(
            "data message for '" << _dc->label() << "' malformed - "
            << reader.getFormattedErrorMessages()
        );
        return;
    }

    // validate chunk
    // TODO: use json schema
    if (!chunk.isMember("id") || !chunk["id"].isString() ||
        !chunk.isMember("total") || !chunk["total"].isUInt() ||
        !chunk.isMember("index") || !chunk["index"].isUInt() ||
        !chunk.isMember("data") || !chunk["data"].isString()) {
        ROS_WARN_STREAM(
            "data message for '" << _dc->label() << "' invalid"
        );
        return;
    }

    // message for chunk
    bool created = false;
    MessagePtr message;
    Messages::iterator i = _messages.find(chunk["id"].asString());
    if (i == _messages.end()) {
        message.reset(new Message(
            chunk["id"].asString(),
            chunk["total"].asUInt(),
            ros::Duration(10 * 60 /* 10 mins*/ )
        ));
        _messages.insert(Messages::value_type(chunk["id"].asString(), message));
        created = true;
    } else {
        message = (*i).second;
    }

    // add chunk to message and finalize if complete
    message->add_chunk(chunk["index"].asUInt(), chunk["data"].asString());
    if (message->is_complete()) {
        _messages.erase(chunk["id"].asString());
        ros_webrtc::Data msg;
        msg.label = _dc->label();
        message->merge(msg);
        _rpub.publish(msg);
    }

}

// ChunkedDataObserver::Message

ChunkedDataObserver::Message::Message(
    const std::string& id,
    size_t count,
    const ros::Duration& duration
    ) : id(id), count(count), expires_at(ros::Time::now() + duration) {
}

void ChunkedDataObserver::Message::add_chunk(size_t index, const std::string& data) {
    for (std::list<Chunk>::iterator i = chunks.begin(); i != chunks.end(); i++) {
        if (index < (*i).index) {
            chunks.insert(i, Chunk(index, data));
            return;
        }
        if (index == (*i).index) {
            return;
        }
    }
    chunks.push_back(Chunk(index, data));
}

void ChunkedDataObserver::Message::merge(ros_webrtc::Data &msg) {
    size_t length = 0;
    for (std::list<Chunk>::const_iterator i = chunks.begin(); i != chunks.end(); i++) {
        length += (*i).buffer.size();
    }
    msg.encoding = "utf-8";
    msg.buffer.reserve(length);
    for (std::list<Chunk>::const_iterator i = chunks.begin(); i != chunks.end(); i++) {
        msg.buffer.insert(
            msg.buffer.end(),
            &(*i).buffer[0],
            &(*i).buffer[0] + (*i).buffer.size()
        );
    }
}

bool ChunkedDataObserver::Message::is_complete() const {
    return chunks.size() == count;
}

bool ChunkedDataObserver::Message::is_expired() const {
    return expires_at < ros::Time::now();
}

// ChunkedDataObserver::Message::Chunk

ChunkedDataObserver::Message::Chunk::Chunk(
    size_t index_,
    const std::string& buffer_
    ) : index(index_) {
    buffer.insert(buffer.end(), &buffer_[0], &buffer_[0] + buffer_.size());
}
