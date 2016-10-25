"""
"""
import time

import bondpy.bondpy
import roslaunch
import rospy
import std_msgs.msg

from ros_webrtc import join_ros_names, normalize_ros_name
import ros_webrtc.msg
import ros_webrtc.srv


class ROSBridge(object):

    @classmethod
    def _node(cls,
            pc,
            data_channel_label,
            wait_for_recv=None,
            **kwargs):
        args = [
            '_session_id:={0}'.format(pc.session_id),
            '_label:={0}'.format(data_channel_label),
            '_peer_id:={0}'.format(pc.peer_id),
        ]
        if pc.namespace:
            args.append('_ros_webrtc_ns:={0}'.format(pc.namespace))
        if wait_for_recv is not None:
            args.append('_wait_for_recv:={0}'.format(wait_for_recv))
        node = roslaunch.core.Node(
            'ros_webrtc',
            'ros_webrtc_rosbridge',
            namespace=rospy.get_namespace(),
            name='rosbridge_{0}_{1}_{2}'.format(*map(normalize_ros_name, [
                pc.session_id, data_channel_label, pc.peer_id
            ])),
            args=' '.join(args),
            **kwargs
        )
        return node

    @classmethod
    def _bond(cls,
            pc,
            data_channel_label,
            **kwargs):
        return bondpy.bondpy.Bond(
            topic='rosbridge_bond',
            id='_'.join([pc.session_id, data_channel_label, pc.peer_id]),
            **kwargs
        )

    def __init__(self,
            pc,
            data_channel_label,
            connect_timeout=None,
            heartbeat_timeout=None,
            on_broken=None,
            on_formed=None,
            **kwargs):
        self.node = self._node(
            pc=pc,
            data_channel_label=data_channel_label,
            **kwargs
        )
        self.process = None
        self.bond = self._bond(
            pc=pc,
            data_channel_label=data_channel_label,
            on_broken=on_broken,
            on_formed=on_formed,
        )
        if connect_timeout is not None:
            self.bond.connect_timeout = connect_timeout
        if heartbeat_timeout is not None:
            self.bond.heartbeat_timeout = heartbeat_timeout

    def __del__(self):
        self.stop()

    def start(self, launch, timeout=None):
        try:
            self.process = launch.launch(self.node)
            self.bond.start()
            if timeout:
                if not self.bond.wait_until_formed(rospy.Duration.from_sec(timeout)):
                    raise RuntimeError(
                        'Bond "{0}" timed out after {1} sec(s).'
                        .format(self.bond.topic, timeout)
                    )
        except Exception:
            self.stop()
            raise

    def stop(self):
        if self.process:
            if self.process.is_alive():
                self.process.stop()
            self.process = None
        if self.bond:
            self.bond.shutdown()
            self.bond = None


class RTCPeerConnection(object):

    def __init__(
            self,
            session_id,
            peer_id,
            namespace=None,
            wait_for_service=None,
            **kwargs):
        self.session_id = session_id
        self.peer_id = peer_id
        self.namespace = namespace or rospy.get_namespace()
        self.cmds = dict([
            (name, rospy.ServiceProxy(join_ros_names(self.namespace, name), srv))
            for name, srv in [
                ('create_peer_connection', ros_webrtc.srv.CreatePeerConnection),
                ('delete_peer_connection', ros_webrtc.srv.DeletePeerConnection),
                ('add_ice_candidate', ros_webrtc.srv.AddIceCandidate),
                ('create_offer', ros_webrtc.srv.CreateOffer),
                ('create_data_channel', ros_webrtc.srv.CreateDataChannel),
                ('set_remote_description', ros_webrtc.srv.SetRemoteDescription),
            ]
        ])
        self._cmd(
            'create_peer_connection',
            wait_for_service=wait_for_service,
            **kwargs
        )

    def __str__(self):
        return '{0}(session_id="{1}", peer_id="{2}")'.format(
            type(self).__name__, self.session_id, self.peer_id,
        )

    def create_offer(self, **kwargs):
        return self._cmd('create_offer', **kwargs)

    def create_data_channel(self, **kwargs):
        return self._cmd('create_data_channel', **kwargs)

    def set_remote_description(self, **kwargs):
        return self._cmd(
            'set_remote_description',
            session_description=ros_webrtc.msg.SessionDescription(**kwargs),
        )

    def add_ice_candidate(self, **kwargs):
        return self._cmd('add_ice_candidate', **kwargs)

    def close(self):
        return self._cmd('delete_peer_connection')

    def bond(self, **kwargs):
        return bond(
            self.session_id,
            self.peer_id,
            namespace=self.namespace,
            **kwargs
        )

    def rosbridge(
            self,
            data_channel_label,
            launch,
            timeout=None,
            **kwargs):
        rospy.loginfo(
            'adapting %s datachannel "%s" to rosbridge',
            self, data_channel_label
        )
        rosbridge = ROSBridge(
            pc=self,
            data_channel_label=data_channel_label,
            **kwargs
        )
        rosbridge.start(launch, timeout=timeout)
        return rosbridge

    def _cmd(self, type_, wait_for_service=None, **kwargs):
        kwargs.update({
            'session_id': self.session_id,
            'peer_id': self.peer_id,
        })
        svc = self.cmds[type_]
        if wait_for_service:
            svc.wait_for_service(wait_for_service)
        return svc(**kwargs)


class RTCPeerConnectionCallbacks(object):

    def __init__(self, pc):

        def _fqn(name):
            return join_ros_names(
                rospy.get_name(),
                'session_{0}'.format(pc.session_id),
                'peer_{0}'.format(pc.peer_id),
                name,
            )

        self.pc = pc
        self.srvs = dict(
            (name, rospy.Service(_fqn(name), srv_cls, handler))
            for name, srv_cls, handler in [
                ('on_add_stream', ros_webrtc.srv.OnAddStream, self.on_add_stream),
                ('on_data_channel', ros_webrtc.srv.OnDataChannel, self.on_data_channel),
                ('on_ice_candidate', ros_webrtc.srv.OnIceCandidate, self.on_ice_candidate),
                ('on_ice_connection_state_change', ros_webrtc.srv.OnIceConnectionStateChange, self.on_ice_connection_state_change),
                ('on_negotiation_needed', ros_webrtc.srv.OnNegotiationNeeded, self.on_negotiation_needed),
                ('on_remove_stream', ros_webrtc.srv.OnRemoveStream, self.on_remove_stream),
                ('on_set_session_description', ros_webrtc.srv.OnSetSessionDescription, self.on_set_session_description),
                ('on_signaling_state_change', ros_webrtc.srv.OnSignalingStateChange, self.on_signaling_state_change),
            ]
        )

    def shutdown(self):
        for srv in self.srvs.values():
            srv.shutdown()

    def on_add_stream(self, req):
        resp = ros_webrtc.srv.OnAddStreamResponse()
        return resp

    def on_data_channel(self, req):
        resp = ros_webrtc.srv.OnDataChannelResponse()
        return resp

    def on_ice_candidate(self, req):
        resp = ros_webrtc.srv.OnIceCandidateResponse()
        return resp

    def on_ice_connection_state_change(self, req):
        resp = ros_webrtc.srv.OnIceConnectionStateChangeResponse()
        return resp

    def on_negotiation_needed(self, req):
        resp = ros_webrtc.srv.OnNegotiationNeededResponse()
        return resp

    def on_remove_stream(self, req):
        resp = ros_webrtc.srv.OnRemoveStreamResponseResponse()
        return resp

    def on_set_session_description(self, req):
        resp = ros_webrtc.srv.OnSetSessionDescriptionResponse()
        return resp

    def on_signaling_state_change(self, req):
        resp = ros_webrtc.srv.OnSignalingStateChangeResponse()
        return resp


class RTCPeerConnectionEvents(object):

    def __init__(self, pc):
        self.pc = pc
        self.subscribers = [
            rospy.Subscriber(
                join_ros_names(
                    pc.namespace,
                    'session_{0}'.format(pc.session_id),
                    'peer_{0}'.format(pc.peer_id),
                    topic,
                ),
                data_cls,
                handler
            )
            for topic, data_cls, handler in [
                ('data_channel', ros_webrtc.msg.DataChannel, self.on_data_channel),
                ('negotiation_needed', std_msgs.msg.Empty, self.on_negotiation_needed),
                ('ice_candidate', ros_webrtc.msg.IceCandidate, self.on_ice_candidate),
                ('ice_candidate_state_change', ros_webrtc.msg.IceCandidateState, self.on_ice_candidate_state_change),
                ('signaling_state_change', ros_webrtc.msg.SignalingState, self.on_signaling_state_change),
                ('add_stream', ros_webrtc.msg.Stream, self.on_add_stream),
                ('remove_stream', ros_webrtc.msg.Stream, self.on_remove_stream),
                ('set_session_description', ros_webrtc.msg.SessionDescription, self.on_set_session_description),
                ('close', std_msgs.msg.Empty, self.on_close),
            ]
        ]

    def wait_for_recv(self, timeout=5.0, poll_freq=1.0):
        rospy.loginfo(
            '%s wait_for_recv - timeout=%0.4f, poll_freq=%0.4f',
            self, timeout, poll_freq,
        )
        stared_at = time.time()
        expire_at = time.time() + timeout
        subscribers = self.subscribers[:]
        while subscribers:
            if subscribers[0].get_num_connections() != 0:
                subscribers.pop(0)
            elif time.time() >= expire_at:
                rospy.loginfo(
                    '%s wait_for_recv - expired after %0.4f sec(s)',
                    self, time.time() - stared_at
                )
                return False
            else:
                time.sleep(poll_freq)
        rospy.loginfo(
            '%s wait_for_recv - succeeded after %0.4f sec(s)',
            self, time.time() - stared_at,
        )
        return True

    def unregister(self):
        for s in self.subscribers:
            s.unregister()

    def on_data_channel(self, req):
        pass

    def on_negotiation_needed(self, req):
        pass

    def on_ice_candidate(self, req):
        pass

    def on_ice_candidate_state_change(self, req):
        pass

    def on_signaling_state_change(self, req):
        pass

    def on_add_stream(self, req):
        pass

    def on_remove_stream(self, req):
        pass

    def on_set_session_description(self, req):
        pass

    def on_close(self, req):
        pass


def bond(session_id, peer_id, namespace=None, **kwargs):
    return bondpy.bondpy.Bond(
        topic=join_ros_names(namespace, 'peer_connection_bond'),
        id='_'.join([session_id, peer_id]),
        **kwargs
    )
