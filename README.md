# ros-webrtc

ROS node for exposing [WebRTC](http://www.webrtc.org/):

- **audio**
- **video**
- **data**

streams using [Google's implementation](https://code.google.com/p/libjingle/).

## dev

```bash
$ git clone git@github.com:ixirobot/ros-webtc.git
$ mkdir ros-webtc-build
$ cd ros-webtc-build
$ cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../ros-webrtc/
```

## deps

Dependencies **not** available through ROS:

- [libjingle-dev](https://github.com/mayfieldrobotics/webrtc-build)
- [ws4py](https://github.com/Lawouach/WebSocket-for-Python)

need to be installed manually:

```bash
$ sudo pip install ws4py
$ cd /tmp && { curl -O "https://s3-us-west-1.amazonaws.com/ai-libjingle-dev/libjingle-dev-1b024da3debug-1_amd64.deb"; cd -; }
$ sudo dpkg -i libjingle-dev-1b024da3debug-1_amd64.deb
```

## usage

`ros-webrtc` **only** exposes WebRTC functionality not tied to a particular use
case. It's up to you to e.g.:

- Describe the session you want (e.g. video and audio tracks, data channels, etc).
- Coordinate signaling (e.g. ICE and SDP message exchange, etc).

Typically your project will implement:

- Signaling ROS service.
- Bridge for moving data to/from a remote peer.

then:

- Register `ros_webrtc` as a `<build_depend/>` and `<run_depend/>`.

and that's it.

## host

This ROS node exposes all WebRTC functionality:

```bash
$ rosrun ros_webrtc host
```

and is configured via `rosparam`, e.g.:

```bash
$ rosparam get /ros_webrtc
cameras:
  downward:
    name: ros:///downward_looking_camera/image_raw
    label: downward
    publish: true
  upward:
    name: ros:///upward_looking_camera/image_raw
    label: upward
    publish: true
    constraints:
      mandatory:
        maxWidth: "640"
        minWidth: "640"
        maxHeight: "480"
        minHeight: "480"
ice_servers:
- {uri: 'stun:stun.services.mozilla.com:3478'}
- {uri: 'stun:stun.l.google.com:19302'}
session:
  constraints:
    optional: {DtlsSrtpKeyAgreement: 'true'}
```

All other configuration is per-session and passed to `host` using `ros_webrtc`
services:

- `srv/BeginSession.srv`
- `srv/ConnectSession.srv`
- `srv/EndSession.srv`
- `srv/AddSessionIceCandidate.srv`
- `srv/SetSessionDescription.srv`

when setting up a connection to a peer. Information about a `host` and its
sessions are here:

- `srv/GetHost.srv`
- `srv/GetSession.srv`


### publishing local media sources

If you've enabled publishing for a source, e.g.:

```bash
$ rosparam get /ros_webrtc/cameras/upward/publish
true
```

then its media (e.g. `sensor_msgs/Image` for video) will be published to a
topic, e.g.:

```bash
$ rostopic show /ros_webrtc/local/upward
Type: sensor_msgs/Image

Publishers: 
 * /webrtc/ros_webrtc_host (http://ai-gazelle:53357/)

Subscribers: None

```

### constraining video sizes

If you need a video source size to be within some range you do so by setting
mandatory constraints on it like e.g.:

```bash
$ rosparam get /webrtc/cameras/upward/
constraints:
  mandatory: {maxHeight: '600', maxWidth: '800', minHeight: '600', minWidth: '800'}
label: upward
name: id:///dev/video0
publish: true
```

which constrains the `upward` video source to have size 800x600.

## tests

```bash
$ catkin_make run_tests
```

and if you want coverage:

```bash
$ catkin_make -DCMAKE_BUILD_TYPE=Debug -DCOVERAGE=ON
$ catkin_make run_tests
$ lcov --path . --directory . --capture --output-file coverage.info
$ lcov --remove coverage.info 'devel/*' 'test/*' '/usr/*' '/opt/*' --output-file coverage.info
$ lcov --list coverage.info
```

## maintainers

* [aisch](https://github.com/aisch)
