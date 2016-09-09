# ros-webrtc [![Build Status](https://travis-ci.org/mayfieldrobotics/ros-webrtc.svg?branch=develop)](https://travis-ci.org/mayfieldrobotics/ros-webrtc) [![codecov](https://codecov.io/gh/mayfieldrobotics/ros-webrtc/branch/develop/graph/badge.svg)](https://codecov.io/gh/mayfieldrobotics/ros-webrtc)

Exposes [Google's implementation](https://code.google.com/p/libjingle/) of
[WebRTC](http://www.webrtc.org/) to ROS. It consists of 2 ROS nodes:

* `ros_webrtc_host`
* `ros_webrtc_rosbridge`

a Python library `ros_webrtc` to help you write your own Python based ROS
WebRTC apps and 2 example ROS nodes:

* `ros_webrtc_example`
* `ros_webrtc_signaling`

which show how everything fits together.

## dev

Get it:

```bash
~/code$ git clone git@github.com:mayfieldrobotics/ros-webrtc.git
```
generate a project if you want, e.g. for Eclipse CDT do:

```bash
~/code$ mkdir ros-webrtc-project
~/code$ cd ros-webrtc-project
~/code/ros-webrtc-project$ cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../ros-webrtc/
```

and setup a catkin workspace using [ansible](http://docs.ansible.com/):

```bash
~$ cd ~/code/ros-webrtc/test/provision
~/code/ros-webrtc/test/provision$ ansible-galaxy install -r requirements.yml
~/code/ros-webrtc/test/provision$ ansible-playbook -i 'localhost,' -c local -K dev.yml
```

or use [vagrant](https://www.vagrantup.com/):

```bash
~$ cd ~/code/ros-webrtc
~/code/ros-webrtc$ vagrant up
```

## test

```bash
~/code/ros-webrtc/test/provision$ cd ~/tmp/ros-webrtc-ws
~/tmp/ros-webrtc-ws$ source devel/setup.bash
~/tmp/ros-webrtc-ws$ catkin_make run_tests
```

and if you want compiled code coverage:

```bash
~/tmp/ros-webrtc-ws$ catkin_make -DCMAKE_BUILD_TYPE=Debug -DCOVERAGE=ON
~/tmp/ros-webrtc-ws$ catkin_make run_tests
```

then use e.g. [lcov](http://ltp.sourceforge.net/coverage/lcov.php) to view the
results:

```
~/tmp/ros-webrtc-ws$ lcov --path . --directory . --capture --no-external --output-file coverage.info
~/tmp/ros-webrtc-ws$ lcov --remove coverage.info 'devel/*' 'test/*' --output-file coverage.info
~/tmp/ros-webrtc-ws$ lcov --list coverage.info
```

## demo

Let's walkthrough an example. Start by launching the demo:

```bash
~$ cd ~/tmp/ros-webrtc-ws
~/tmp/ros-webrtc-ws$ source devel/setup.bash
~/tmp/ros-webrtc-ws$ roslaunch ros_webrtc ros_webrtc.launch
...
```

If your webcam is somewhere other than `/dev/video0` (e.g. `/dev/video2`)
set that when launching:

```bash
~/tmp/ros-webrtc-ws$ ROS_WEBRTC_WEBCAM=id:///dev/video2 roslaunch ros_webrtc ros_webrtc.launch
```

This all leaves you with a few `ros_webrtc` nodes:

```bash
~/tmp/ros-webrtc-ws$ rosnode list
/ros_webrtc/host
/ros_webrtc_example/example
/ros_webrtc_example/signaling
/rosout
```

### /ros_webrtc/host

This `ros_webrtc_host` instance exposes a [build](https://github.com/mayfieldrobotics/webrtc-build)
of [Google's implementation](https://code.google.com/p/libjingle/) of WebRTC to
ROS using an interface similar to [the one exposed to browsers](https://developer.mozilla.org/en-US/docs/Web/Guide/API/WebRTC).

So to e.g. create an [RTCPeerConnection](https://developer.mozilla.org/en-US/docs/Web/API/RTCPeerConnection)
you'd use this `ros_webrtc_host` ROS service:

```bash
~/tmp/ros-webrtc-ws$ rosservice info /ros_webrtc/create_peer_connection
Node: /ros_webrtc/host
URI: rosrpc://ai-gazelle:34929
Type: ros_webrtc/CreatePeerConnection
Args: session_id peer_id sdp_constraints video_sources audio_sources
```

which has type `ros_webrtc/CreatePeerConnection`:

```bash
~/tmp/ros-webrtc-ws$ rossrv info ros_webrtc/CreatePeerConnection
rossrv show ros_webrtc/CreatePeerConnection
string session_id
string peer_id
ros_webrtc/MediaConstraints sdp_constraints
  ros_webrtc/Constraint[] mandatory
    string key
    string value
  ros_webrtc/Constraint[] optional
    string key
    string value
string[] video_sources
string[] audio_sources
---
```

### /ros_webrtc_example/signaling

Initiating a peer connection and negotiating how and what to send over it is
referred to as *signaling* in WebRTC. You can do that anyway you like (e.g. an
internet facing pub/sub service) as long that the peer's can send each other
signaling messages.

The example ROS node `ros_webrtc_signaling` is a simple WebSocket sever that
does that and is suitable for testing. The peers (e.g. a browser and a
`ros_webrtc_example` ROS node) will **both** connect to
`ros_webrtc_signaling` and exchange signaling messages.

Just as the browser [RTCPeerConnection](https://developer.mozilla.org/en-US/docs/Web/API/RTCPeerConnection)
has no knowledge of signaling, neither does `ros_webrtc_host`.

### /ros_webrtc_example/example

A `ros_webrtc_example` ROS node intended to show how ROS WebRTC app *might*
be implemented using helpers from the `ros_webrtc` Python library.

It connects to [signaling](#ros_webrtcsignaling) and can be *called* by another peer. In
that case a signaling message will be **received** from the other peer via
[signaling](#ros_webrtcsignaling).

Alternatively `ros_webrtc_example` can *call* another peer. In that case
signaling message will be **sent** from the other peer via
[signaling](#ros_webrtcsignaling).

### /ros_webrtc/rosbridge_*

You shouldn't see instances of the `ros_webrtc_rosbridge` ROS node yet as
they tunnel the ROS protocol over an [RTCDataChannel](https://developer.mozilla.org/en-US/docs/Web/API/RTCDataChannel)
using [rosbridge](http://wiki.ros.org/rosbridge-depricated).

### test site

In another terminal [serve the test site](http://localhost:8080) which we'll use
later to setup a peer connection between the browser and `ros_webrtc`:

```bash
~$ cd ~/code/ros-webrtc/test/fixtures/site
~/code/ros-webrtc/test/fixtures/site$ grunt clean build connect watch
...
```

### connect!

Finally, it's time to connect. Let start by initiating a peer connection to
`ros_webrtc_example` from a [compatible browser](http://iswebrtcreadyyet.com/)
using the [test site](#test-site):

```
http://localhost:8080/#cd0bdd0b147c4ba4bea16ca3cb35c140/call/{host-name}
```

where:

* `cd0bdd0b147c4ba4bea16ca3cb35c140` is a random identifier for the browser
* `{host-name}` is the host name of your local machine (e.g. whatever
  `$ hostname` prints).

Once the connection is established `ros_webrtc_host` will list it:

```bash
~/tmp/ros-webrtc-ws$ rosservice call /ros_webrtc/get_host
...
peer_connections: 
  - 
    session_id: 9eb17a42bbf947649f3b92f24fc7b88c
    peer_id: cd0bdd0b147c4ba4bea16ca3cb35c140
...
```

Notice that a peer connection is identified by **2** values:

* a `session_id` and 
* a `peer_id`

We also have a new `ros_webrtc_rosbridge` node instance:

```bash
~/tmp/ros-webrtc-ws$ rosnode list /ros_webrtc
...
/ros_webrtc/rosbridge_9eb17a42bbf947649f3b92f24fc7b88c_rosbridge_cd0bdd0b147c4ba4bea16ca3cb35c140
...
```

which is tunneling the ROS protocol over this peer connection's `rosbridge`
data channel . We also have a bunch of new peer connection specific service
advertised by `ros_webrtc_example`:

```
~/tmp/ros-webrtc-ws$ rosservice list /ros_webrtc_example/example/session_aa2f63408bb444508d68bc8a60dae20c/peer_cd0bdd0b147c4ba4bea16ca3cb35c140/
/ros_webrtc_example/example/session_aa2f63408bb444508d68bc8a60dae20c/peer_cd0bdd0b147c4ba4bea16ca3cb35c140/on_add_stream
/ros_webrtc_example/example/session_aa2f63408bb444508d68bc8a60dae20c/peer_cd0bdd0b147c4ba4bea16ca3cb35c140/on_close
/ros_webrtc_example/example/session_aa2f63408bb444508d68bc8a60dae20c/peer_cd0bdd0b147c4ba4bea16ca3cb35c140/on_data_channel
/ros_webrtc_example/example/session_aa2f63408bb444508d68bc8a60dae20c/peer_cd0bdd0b147c4ba4bea16ca3cb35c140/on_ice_candidate
/ros_webrtc_example/example/session_aa2f63408bb444508d68bc8a60dae20c/peer_cd0bdd0b147c4ba4bea16ca3cb35c140/on_ice_connection_state_change
/ros_webrtc_example/example/session_aa2f63408bb444508d68bc8a60dae20c/peer_cd0bdd0b147c4ba4bea16ca3cb35c140/on_negotiation_needed
/ros_webrtc_example/example/session_aa2f63408bb444508d68bc8a60dae20c/peer_cd0bdd0b147c4ba4bea16ca3cb35c140/on_remove_stream
/ros_webrtc_example/example/session_aa2f63408bb444508d68bc8a60dae20c/peer_cd0bdd0b147c4ba4bea16ca3cb35c140/on_set_session_description
/ros_webrtc_example/example/session_aa2f63408bb444508d68bc8a60dae20c/peer_cd0bdd0b147c4ba4bea16ca3cb35c140/on_signaling_state_change
```

and invoked by `ros_webrtc_host` as calbacks which mirror [RTCPeerConnection event handlers](https://developer.mozilla.org/en-US/docs/Web/API/RTCPeerConnection#Event_handlers). Note also that these events are also
published to topics which you can monitor:

```bash
~/tmp/ros-webrtc-ws$ rostopic list /ros_webrtc/
/ros_webrtc/add_stream
/ros_webrtc/chatter
/ros_webrtc/close
/ros_webrtc/data_channel
/ros_webrtc/ice_candidate
/ros_webrtc/ice_connection_state_change
/ros_webrtc/local/webcam
/ros_webrtc/negotiation_needed
/ros_webrtc/peer_connection_bond
/ros_webrtc/remove_stream
/ros_webrtc/session_aa2f63408bb444508d68bc8a60dae20c/peer_cd0bdd0b147c4ba4bea16ca3cb35c140/data_rosbridge
/ros_webrtc/set_session_description
/ros_webrtc/signaling_state_change
```

We can **reverse** the direction of the call and have `ros_webrtc_example`
call the browser. Change the browser to:

```
http://localhost:8080/#cd0bdd0b147c4ba4bea16ca3cb35c140/wait
```

and call it from `ros_webrtc_example`:

```bash
~/tmp/ros-webrtc-ws$ rosservice call /ros_webrtc_example/example/call cd0bdd0b147c4ba4bea16ca3cb35c140 ''
```

where:

* `cd0bdd0b147c4ba4bea16ca3cb35c140` is the browser peer id to call and
* `''` is an empty string indicating `ros_webrtc_example` should generate a
  random session id for the peer connection

At this point read the commented `ros_webrtc_example` source to get an idea
of how these connections were established and what a ROS WebRTC app looks like.

# params

These are used to configure `ros_webrtc_host`, e.g.:

```bash
~/tmp/ros-webrtc-ws$ rosparam get /ros_webrtc
cameras:
  webcam:
    constraints:
      mandatory: {maxHeight: '480', maxWidth: '640', minHeight: '480', minWidth: '640'}
    label: webcam
    name: id:///dev/video0
    publish: true
example: {id: hostname}
ice_servers:
- {uri: 'stun:stun.services.mozilla.com:3478'}
- {uri: 'stun:stun.l.google.com:19302'}
queue_sizes: {audio: 1000, data: 1000, event: 1000, video: 1000}
session:
  constraints:
    optional: {DtlsSrtpKeyAgreement: 'true'}
trace: {file: /tmp/ros_webrtc.trace, filter: all}
```

## cameras

This is a mapping describing the video sources of the host. Each has these
fields:

* `label` - string naming the video source.
* `constraints`- optional - nested mapping of `mandatory` and `optional`
  **string** constraints.
* `name` - string of the form `{type}://{resource}`.
* `publish` - optional - boolean controlling whether images from this source
  should be published.

See [Google's source](https://chromium.googlesource.com/external/webrtc/+/master/webrtc/api/mediaconstraintsinterface.cc)
for possible `constraints`. In the e.g. above we constrained the `webcam`
source to have size `640x480`.

The `name` determines where video source images come from and is one of these
*types*:

* `name` - **name** of the video source (e.g. `name://BisonCam, NB Pro`).
* `id` - device file of the video source (e.g. `id:///dev/video0`).
* `ros` - a ROS [sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)
  topic (e.g. `ros:///my/image/topic`).

If `publish` is true then `ros_webrtc_host` will publish source images to a
[sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)
topic (e.g. `/ros_webrtc/local/webcam`).

## ice_servers

A list of [STUN and TURN](http://www.html5rocks.com/en/tutorials/webrtc/infrastructure/)
servers to use when generating [ice candidates](https://developer.mozilla.org/en-US/docs/Web/API/RTCIceCandidate).
Each is a mapping with fields:

- `uri` - type and network location formatted as `{sturn|turn}:{hostname}[:{port}]`.
- `username` - optional - authentication.
- `password` - optional - authentication.

## trace

A mapping with these fields controlling [libjingle](https://code.google.com/p/libjingle/)
tracing:

* `file` - path to file where traces should be written.
* `filter` - optional - either a sequence of strings or a single string of [trace levels] (https://chromium.googlesource.com/external/webrtc/+/master/webrtc/common_types.h) used to build a filter. In the e.g. above `all`
   traces have been included.

## queue_sizes

Mapping of queue sizes to use for `ros_webrtc_example` ROS publishers. Each
field of the mapping if a type of publisher (`audio`, `video`, `data` and
`event`) with an integer value for the queue size.

### peer_connection/timeout

Number of seconds (as a `double`) to use for the peer connection [bond](http://wiki.ros.org/bond)
heart-beat timeout, or `0` to disable peer connection bonding (useful when
debugging).

A peer connection bond is typically formed between the local WebRTC application
and `ros_webrtc_host` so that either side is notified if peer connection is
closed *out-of-band*.

### peer_connection/constraint

This is a nested mapping of `mandatory` and `optional` **string** constraints
to apply to each peer connection. See [Google's source](https://chromium.googlesource.com/external/webrtc/+/master/webrtc/api/mediaconstraintsinterface.cc) for possible `constraints`. In the e.g. above we
constrained peer connections to optionally enable [DTLS-SRTP for key management](https://webrtchacks.com/webrtc-must-implement-dtls-srtp-but-must-not-implement-sdes/).

### open_media_sources

Boolean controlling whether local media sources (cameras, microphones, speakers,
etc) are opened/captured on start. It is `true` by default.

If set to `false` then local media sources will only be opened when there are
peer connections that need them. So e.g. they will be opened on first peer
connection, closed when the last peer connection ends then re-opened when
the next peer connection is created.
