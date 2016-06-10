#include "convert.h"

ros_webrtc::Track to_ros(const webrtc::MediaStreamTrackInterface* src) {
    ros_webrtc::Track dst;
    dst.kind = src->kind();
    dst.id = src->id();
    switch (src->state()) {
        case webrtc::MediaStreamTrackInterface::TrackState::kLive:
            dst.state = "live";
            break;
        case webrtc::MediaStreamTrackInterface::TrackState::kEnded:
            dst.state = "ended";
            break;
    }
    dst.enabled = src->enabled();
    return dst;
}

const char *to_string(webrtc::PeerConnectionInterface::SignalingState src) {
    switch (src) {
        case webrtc::PeerConnectionInterface::SignalingState::kStable:
            return "stable";
        case webrtc::PeerConnectionInterface::SignalingState::kHaveLocalOffer:
            return "have_local_offer";
        case webrtc::PeerConnectionInterface::SignalingState::kHaveLocalPrAnswer:
            return "have_local_pr_answer";
        case webrtc::PeerConnectionInterface::SignalingState::kHaveRemoteOffer:
            return "have_remote_offer";
        case webrtc::PeerConnectionInterface::SignalingState::kHaveRemotePrAnswer:
            return "have_remote_pr_answer";
            break;
        case webrtc::PeerConnectionInterface::SignalingState::kClosed:
            return "closed";
    };
    return "unknown";
}

const char *to_string(webrtc::PeerConnectionInterface::IceConnectionState src) {
    switch (src) {
        case webrtc::PeerConnectionInterface::IceConnectionState::kIceConnectionNew:
            return "new";
        case webrtc::PeerConnectionInterface::IceConnectionState::kIceConnectionChecking:
            return "checking";
        case webrtc::PeerConnectionInterface::IceConnectionState::kIceConnectionConnected:
            return "connected";
        case webrtc::PeerConnectionInterface::IceConnectionState::kIceConnectionCompleted:
            return "completed";
        case webrtc::PeerConnectionInterface::IceConnectionState::kIceConnectionFailed:
            return "failed";
        case webrtc::PeerConnectionInterface::IceConnectionState::kIceConnectionDisconnected:
            return "disconnected";
        case webrtc::PeerConnectionInterface::IceConnectionState::kIceConnectionClosed:
            return "closed";
    };
    return "unknown";
}

const char *to_string(webrtc::PeerConnectionInterface::IceGatheringState src) {
    switch (src) {
        case webrtc::PeerConnectionInterface::IceGatheringState::kIceGatheringNew:
            return "new";
        case webrtc::PeerConnectionInterface::IceGatheringState::kIceGatheringGathering:
            return "gathering";
        case webrtc::PeerConnectionInterface::IceGatheringState::kIceGatheringComplete:
            return "complete";
    }
    return "unknown";
}

const char *to_string(webrtc::DataChannelInterface::DataState src) {
    switch (src) {
        case webrtc::DataChannelInterface::DataState::kConnecting:
            return "connecting";
        case webrtc::DataChannelInterface::DataState::kOpen:
            return "open";
        case webrtc::DataChannelInterface::DataState::kClosing:
            return "closing";
        case webrtc::DataChannelInterface::DataState::kClosed:
            return "closed";
    }
    return "unknown";
}

const char *to_string(webrtc::MediaSourceInterface::SourceState src) {
    switch (src) {
        case webrtc::MediaSourceInterface::kInitializing:
            return "initializing";
        case webrtc::MediaSourceInterface::kLive:
            return "live";
        case webrtc::MediaSourceInterface::kEnded:
            return "ended";
        case webrtc::MediaSourceInterface::kMuted:
            return "muted";
        default:
            return "unknown";
    }
}
