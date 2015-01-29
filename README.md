ros-webrtc
==========

ROS node for managing [WebRTC](http://www.webrtc.org/):

- **audio**
- **video**
- **data**

streaming sessions using [Google's implementation](https://code.google.com/p/libjingle/).

dev
===

```bash
$ git clone git@github.com:ixirobot/ros-webtc.git
$ mkdir ros-webtc-build
$ cd ros-webtc-build
$ cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../ros-webtc/
```

usage
=====

`ros-webrtc` intentionally exposes **only** use-case agnostic WebRTC
functionality.


It's up to you to e.g.:

- Describe the session you want (e.g. video and audio tracks, data channels, etc).
- Coordinate signaling (e.g. ICE and SDP message exchange, etc).

so typically your project will implement:

- Ingress (from remote peer) signaling ROS service(s).
- Egress (from local device) signaling ROS service(s)
- Data-channel bridge as a ROS publisher/subscriber for moving data to/from remote peer and local device.

then:

- Register `ros-webrtc` as a `<build_depend/>` and `<run_depend/>`.

and that's it.

Here's e.g. of an **ingress** signaling implementation linking
[pubnub](http://www.pubnub.com/) signals to the corresponding `ros_webrtc` ROS
service(s):

```python
import rosgraph
import rospy

import ros_webrtc


class SignalSubscriber(object):
    
    def __init__(self):
        self.srvs = dict(
            (name, rospy.ServiceProxy(rosgraph.names.ns_join('ros_webrtc', name), srv))
            for name, srv in [
                ('connect', ros_webrtc.srv.Connect),
                ('disconnect', ros_webrtc.srv.Disconnect),
                ('ice_candidate', ros_webrtc.srv.IceCandidate),
                ('sdp_offer_answer', ros_webrtc.srv.SdpOfferAnswer),
            ]
        )
    
    def __call__(self, message, channel):
        try:
            rospy.loginfo('pubnub @ %s - %s', channel, message)
            if not isinstance(message, collections.Mapping):
                rospy.logwarn('pubnub @ %s is not a mapping - %s', channel, message)
                return
            if 'type' not in message:
                rospy.logwarn('pubnub @ %s has no "type" key - %s', channel, message)
                return
            if 'payload' not in message:
                rospy.logwarn('pubnub @ %s has no "payload" key - %s', channel, message)
                return
            if message['type'] == 'connect':
                handler = self._on_connect
            elif message['type'] == 'disconnect':
                handler = self._on_disconnect
            elif message['type'] == 'ice_candidate':
                handler = self._on_ice_candidate
            elif message['type'] == 'sdp_offeranswer':
                handler = self._on_sdp_offeranswer
            else:
                rospy.loginfo('pubnub @ %s has unsupported "type" - %s', channel, message)
                return
            handler(channel, message)
        except Exception, ex:
            rospy.logerr(ex)
    
    def _on_connect(self, channel, message):
        self.srvs['connect'](
            peer_id=message['device'],
            sdp_constraints=ros_webrtc.msg.MediaConstraints(
                mandatory=[],
                optional=[],
            ),
            data_channels=[
                ros_webrtc.msg.DataChannel(
                    label='rosbridge',
                    id=-1,
                    reliable=False,
                    ordered=False,
                ),
            ],
            disconnect_service=rosgraph.names.ns_join(NAMESPACE, 'disconnect'),
            ice_candidate_service=rosgraph.names.ns_join(NAMESPACE, 'ice_candidate'),
            sdp_offer_answer_service=rosgraph.names.ns_join(NAMESPACE, 'sdp_offer_answer'),
        )
    
    def _on_disconnect(self, channel, message):
        self.srvs['disconnect'](
            peer_id=message['device'],
        )
    
    def _on_ice_candidate(self, channel, message):
        self.srvs['ice_candidate'](
            peer_id=message['device'],
            sdp_mid=message['payload']['sdpMid'],
            sdp_mline_index=message['payload']['sdpMLineIndex'],
            candidate=message['payload']['candidate'],
        )
    
    def _on_sdp_offeranswer(self, channel, message):
        self.srvs['sdp_offer_answer'](
            peer_id=message['device'],
            type=message['payload']['type'],
            sdp=message['payload']['sdp'],
        )
```

device
------

This ROS node exposes all WebRTC functionality:

```bash
$ rosrun ros_webrtc device
```

and is configured via `rosparam`:

```bash
$ rosparam get /ros_webrtc
cameras:
  downward: {label: downward, name: Loopback video device 0}
  upward: {label: upward, name: Loopback video device 1}
ice_servers:
- {uri: 'stun:stun.services.mozilla.com:3478'}
- {uri: 'stun:stun.l.google.com:19302'}
session:
  constraints:
    optional: {DtlsSrtpKeyAgreement: 'true'}
```

All other configuration is per-session and passed to `device` via `ros_webrtc`
services:

- `srv/Connect.srv`.
- `srv/Disconnect.srv`.
- `srv/IceCandidate.srv`.
- `srv/SdpOfferAnswer.srv`.
