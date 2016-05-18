#ifndef ROS_WEBRTC_CONVERT_H_
#define ROS_WEBRTC_CONVERT_H_

#include <ros_webrtc/Track.h>
#include <talk/app/webrtc/peerconnectioninterface.h>

ros_webrtc::Track to_ros(const webrtc::MediaStreamTrackInterface *src);

const char *to_string(webrtc::PeerConnectionInterface::SignalingState src);

const char *to_string(webrtc::PeerConnectionInterface::IceConnectionState src);

const char *to_string(webrtc::PeerConnectionInterface::IceGatheringState src);

const char *to_string(webrtc::DataChannelInterface::DataState src);

const char *to_string(webrtc::MediaSourceInterface::SourceState src);

#endif /* ROS_WEBRTC_CONVERT_H_ */
