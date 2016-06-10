#ifndef ROS_WEBRTC_DATA_CHANNEL_H_
#define ROS_WEBRTC_DATA_CHANNEL_H_

#include <ros/ros.h>
#include <ros_webrtc/Data.h>
#include <ros_webrtc/DataChannel.h>
#include <webrtc/api/datachannelinterface.h>
#include <webrtc/base/scoped_ref_ptr.h>

#include "media_type.h"
#include "renderer.h"

class DataChannel {

public:

    DataChannel(
        ros::NodeHandle &nh,
        const std::string& recv_topic,
        webrtc::DataChannelInterface *provider,
        const MediaType& media_type,
        size_t queue_size=1000);

    void send(const ros_webrtc::Data& msg);

    void send(webrtc::DataBuffer& data_buffer);

    bool is_chunked() const;

    size_t chunk_size() const;

    operator ros_webrtc::DataChannel () const;

    size_t reap();

private:

    rtc::scoped_refptr<webrtc::DataChannelInterface> _provider;
    MediaType _media_type;
    DataObserverPtr _data_observer;

};

typedef boost::shared_ptr<DataChannel> DataChannelPtr;

typedef boost::shared_ptr<const DataChannel> DataChannelConstPtr;

#endif /* ROS_WEBRTC_DATA_CHANNEL_H_ */
