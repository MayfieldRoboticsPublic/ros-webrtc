#include <json/json.h>

#include "data_channel.h"
#include "util.h"

// ChunkedDataTransfer

struct ChunkedDataTransfer {

    ChunkedDataTransfer(
        const std::string& id,
        const webrtc::DataBuffer& data_buffer,
        size_t size
    );

    bool is_complete() const;

    size_t operator()(webrtc::DataChannelInterface* provider);

    const std::string id;

    const rtc::CopyOnWriteBuffer& data;

    const size_t size;

    const size_t total;

    size_t current;

};

ChunkedDataTransfer::ChunkedDataTransfer(
    const std::string& id,
    const webrtc::DataBuffer& data_buffer,
    size_t size) :
    id(id),
    data(data_buffer.data),
    size(size),
    current(0),
    total(static_cast<size_t>(std::ceil((double)data.size() / (double)size))) {
}

bool ChunkedDataTransfer::is_complete() const {
    return current == total;
}

size_t ChunkedDataTransfer::operator()(webrtc::DataChannelInterface* provider) {
    size_t bytes = 0;

    Json::Value chunk;
    chunk["id"] = id;
    chunk["index"] = static_cast<Json::UInt>(current);
    chunk["total"] = static_cast<Json::UInt>(total);
    chunk["data"] = std::string(
        (const char *)(&data.data()[0] + current * size),
        std::min(size, data.size() - current * size)
    );
    std::string serialized = chunk.toStyledString();
    webrtc::DataBuffer data_buffer(serialized);
    provider->Send(data_buffer);
    bytes += data_buffer.size();
    current++;

    return bytes;
}

// DataChannel

DataChannel::DataChannel(
    ros::NodeHandle& nh,
    const std::string& recv_topic,
    webrtc::DataChannelInterface *provider,
    const MediaType& media_type,
    size_t queue_size) :
        _provider(provider),
        _media_type(media_type) {
    if (is_chunked()) {
        _data_observer.reset(new ChunkedDataObserver(
            nh,
            recv_topic,
            queue_size,
            provider
        ));
    } else {
        _data_observer.reset(new UnchunkedDataObserver(
            nh,
            recv_topic,
            queue_size,
            provider
        ));
    }
}

bool DataChannel::is_chunked() const {
    return chunk_size() != 0;
}

size_t DataChannel::chunk_size() const {
    auto i = _media_type.params.find("chunksize");
    return i == _media_type.params.end() ? 0 : std::atoi((*i).second.c_str());
}

void DataChannel::send(const ros_webrtc::Data& msg) {
    webrtc::DataBuffer data_buffer(
        rtc::CopyOnWriteBuffer(&msg.buffer[0], msg.buffer.size()),
        msg.encoding == "binary"
    );
    send(data_buffer);
}

void DataChannel::send(webrtc::DataBuffer& data_buffer) {
    if (!is_chunked()) {
        _provider->Send(data_buffer);
    } else {
        ChunkedDataTransfer xfer(generate_id(), data_buffer, chunk_size());
        while (!xfer.is_complete()) {
            // TODO: rate limit?
            xfer(_provider);
        }
    }
}

DataChannel::operator ros_webrtc::DataChannel () const {
    ros_webrtc::DataChannel dst;
    dst.label = _provider->label();
    dst.id = _provider->id();
    dst.reliable = _provider->reliable();
    dst.ordered = _provider->ordered();
    dst.protocol = _provider->protocol();
    dst.chunk_size = chunk_size();
    dst.state = _provider->state();
    return dst;
}

size_t DataChannel::reap() {
    return _data_observer->reap();
}
