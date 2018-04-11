"""
"""
import functools
import weakref

import rospy

import ros_webrtc.srv

from ros_webrtc import join_ros_names
from ros_webrtc.msg import IceServer
from ros_webrtc.peer_connection import (
    RTCPeerConnection,
    RTCPeerConnectionCallbacks,
)


class ApplicationRTCPeerConnectionCallbacks(RTCPeerConnectionCallbacks):

    def __init__(self, app, pc):
        self.app = weakref.proxy(app)
        super(ApplicationRTCPeerConnectionCallbacks, self).__init__(pc)

    def on_data_channel(self, req):
        if not self.app.on_pc_data_channel(self.pc, req.data_channel):
            return
        return super(ApplicationRTCPeerConnectionCallbacks, self).on_data_channel(req)

    def on_ice_candidate(self, req):
        self.app.on_pc_ice_candidate(self.pc, req.candidate)
        return super(ApplicationRTCPeerConnectionCallbacks, self).on_ice_candidate(req)

    def on_ice_candidate_state_change(self, req):
        self.app.on_pc_ice_candidate_state_change(self.pc, req)
        return super(ApplicationRTCPeerConnectionCallbacks, self).on_ice_candidate_state_change(req)

    def on_signaling_state_change(self, req):
        self.app.on_pc_signaling_state_change(self.pc, req)
        return super(ApplicationRTCPeerConnectionCallbacks, self).on_signaling_state_change(req)

    def on_negotiation_needed(self, req):
        self.app.on_pc_negotiation_needed(self.pc, req)
        return super(ApplicationRTCPeerConnectionCallbacks, self).on_negotiation_needed(req)

    def on_add_stream(self, req):
        self.app.on_pc_add_stream(self.pc, req.stream)
        return super(ApplicationRTCPeerConnectionCallbacks, self).on_add_stream(req)

    def on_remove_stream(self, req):
        self.app.on_pc_remove_stream(self.pc, req.stream)
        return super(ApplicationRTCPeerConnectionCallbacks, self).on_remove_stream(req)

    def on_set_session_description(self, req):
        self.app.on_pc_set_session_description(self.pc, req)
        return super(ApplicationRTCPeerConnectionCallbacks, self).on_set_session_description(req)


class ApplicationRTCPeerConnection(RTCPeerConnection):

    def __init__(
            self,
            app,
            session_id,
            peer_id,
            bond_connect_timeout=None,
            bond_heartbeat_timeout=None,
            **kwargs):
        super(ApplicationRTCPeerConnection, self).__init__(
            session_id, peer_id, **kwargs
        )
        self.app = weakref.proxy(app)
        if bond_connect_timeout != 0 and bond_heartbeat_timeout != 0:
            self.pc_bond = self.bond(
                on_broken=self.on_pc_bond_broken,
                on_formed=self.on_pc_bond_formed,
            )
            if bond_connect_timeout is not None:
                self.pc_bond.connect_timeout = bond_connect_timeout
            if bond_heartbeat_timeout is not None:
                self.pc_bond.heartbeat_timeout = bond_heartbeat_timeout
        else:
            rospy.loginfo('%s bonding disabled', self)
            self.pc_bond = None
        self.rosbridges = []
        self.deleting = False
        self.callbacks = ApplicationRTCPeerConnectionCallbacks(app, self)
        self.app.pcs[(self.session_id, self.peer_id)] = self
        if self.pc_bond:
            self.pc_bond.start()

    def delete(self):
        rospy.loginfo('%s deleting', self)
        if self.app is None:
            return
        self.app, app = None, self.app
        try:
            app.on_pc_delete(self)
        finally:
            try:
                if self.pc_bond:
                    self.pc_bond.shutdown()
                    self.pc_bond = None
                while self.rosbridges:
                    rosbridge = self.rosbridges.pop()
                    rosbridge.stop()
                if self.callbacks:
                    self.callbacks.shutdown()
                try:
                    self.close()
                except rospy.ServiceException, ex:
                    rospy.loginfo('%s close cmd failed - %s', self, ex)
            finally:
                app.pcs.pop((self.session_id, self.peer_id), None)
                self.deleting = False

    def on_pc_bond_broken(self):
        rospy.loginfo('%s bond broken, deleting pc ...', self)
        self.delete()

    def on_pc_bond_formed(self):
        rospy.loginfo('%s bond formed', self)

    def on_rosbridge_bond_broken(self, label):
        rospy.loginfo('%s rosbridge "%s" bond broken, deleting pc ...', self, label)
        self.delete()

    def on_rosbridge_bond_formed(self, label):
        rospy.loginfo('%s rosbridge "%s" bond formed', self, label)

    # RTCPeerConnection

    def rosbridge(self, data_channel_label, launch, timeout=5.0, **kwargs):
        rosbridge = super(ApplicationRTCPeerConnection, self).rosbridge(
            data_channel_label,
            launch,
            on_broken=functools.partial(
                self.on_rosbridge_bond_broken, data_channel_label
            ),
            on_formed=functools.partial(
                self.on_rosbridge_bond_formed, data_channel_label
            ),
            timeout=timeout,
            **kwargs
        )
        self.rosbridges.append(rosbridge)
        return rosbridge


class Application(object):

    def __init__(self,
            id_,
            ros_webrtc_namespace=None,
            pc_connect_timeout=None,
            pc_heartbeat_timeout=None):
        self.id = id_
        self.pcs = {}
        self.svrs = []
        self.ros_webrtc_namespace = ros_webrtc_namespace
        self.pc_connect_timeout = pc_connect_timeout
        self.pc_heartbeat_timeout = pc_heartbeat_timeout
        self.ice_server_svc = rospy.ServiceProxy(
            join_ros_names(self.ros_webrtc_namespace, 'set_ice_servers'),
            ros_webrtc.srv.SetIceServers
        )

    def shutdown(self):
        for pc in self.pcs.values():
            pc.delete()
        for srv in self.svrs:
            srv.shutdown()

    def set_ice_servers(self, ice_servers):
        valid_servers = []
        for server in ice_servers:
            if 'uri' not in server:
                continue

            if 'stun' in server['uri'] or \
                    all(k in server for k in ('username', 'password')):
                valid_servers.append(
                    IceServer(uri=server['uri'],
                              username=server.get('username', ''),
                              password=server.get('password', ''))
                )

        if not valid_servers:
            rospy.logerr("No properly formatted ice servers found in {}".
                         format(ice_servers))
            return

        rospy.loginfo("Setting ice servers: {}".format(valid_servers))
        return self.ice_server_svc(valid_servers)

    def create_pc(self, session_id, peer_id, **kwargs):
        key = (session_id, peer_id)
        pc = self.pcs.pop(key, None)
        if pc:
            rospy.loginfo('pc %s already exists, deleting ... ', pc)
            pc.delete()
        if self.ros_webrtc_namespace:
            kwargs['namespace'] = self.ros_webrtc_namespace
        pc = ApplicationRTCPeerConnection(
            self,
            session_id,
            peer_id,
            bond_connect_timeout=self.pc_connect_timeout,
            bond_heartbeat_timeout=self.pc_heartbeat_timeout,
            **kwargs
        )
        rospy.loginfo('created pc %s', pc)
        return pc

    def get_pc(self, session_id, peer_id):
        key = (session_id, peer_id)
        return self.pcs.get(key)

    def delete_pc(self, session_id, peer_id):
        key = (session_id, peer_id)
        pc = self.pcs.pop(key, None)
        if pc:
            pc.delete()

    def on_pc_delete(self, pc):
        pass

    def on_pc_data_channel(self, pc, msg):
        pass

    def on_pc_negotiation_needed(self, pc, msg):
        pass

    def on_pc_ice_candidate(self, pc, msg):
        pass

    def on_pc_ice_candidate_state_change(self, pc, msg):
        pass

    def on_pc_signaling_state_change(self, pc, msg):
        pass

    def on_pc_add_stream(self, pc, msg):
        pass

    def on_pc_set_session_description(self, pc, msg):
        pass
