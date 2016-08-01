#!/usr/bin/env python
import binascii
import collections
import os
import random
import time
import unittest
import uuid
import weakref

import rosgraph
import rospy
import rostest
import sensor_msgs.msg

import ros_webrtc.application as app
import ros_webrtc.msg
import ros_webrtc.signaling as sig
import ros_webrtc.srv

from ros_coverage import ros_coverage


PKG = 'ros_webrtc'
NAME = 'session_test'


class Application(app.Application):

    class SignalHandler(sig.SignalHandler):

        def __init__(self, app):
            self.app = weakref.proxy(app)
            super(Application.SignalHandler, self).__init__()

        def _pc_for(self, channel, session):
            pc = self.app.pcs.get((session, channel))
            if not pc:
                raise sig.DropSignal('no pc')
            return pc

        # sig.SignalHandler

        def on_call(self, channel, session, payload):
            self.app.create_pc(session, channel,
                sdp_constraints=ros_webrtc.msg.MediaConstraints(
                    mandatory=[],
                    optional=[],
                ),
                video_sources=['*'],
                audio_sources=['*'],
            )
            self.app.sig_cli.send('pickup', channel, session)

        def on_add_ice_candidate(self, channel, session, payload):
            pc = self._pc_for(channel, session)
            pc.add_ice_candidate(
                sdp_mid=payload['sdpMid'],
                sdp_mline_index=payload['sdpMLineIndex'],
                candidate=payload['candidate'],
            )

        def on_set_session_description(self, channel, session, payload):
            pc = self._pc_for(channel, session)
            pc.set_remote_description(
                type=payload['type'],
                sdp=payload['sdp'],
            )

        def on_hangup(self, channel, session, payload):
            pc = self._pc_for(channel, session)
            pc.delete()


    def __init__(self, signaling_url):
        super(Application, self).__init__('test_{0}'.format(uuid.uuid4().hex))
        self.sig_cli = sig.SignalClient(
            signaling_url + '/' + self.id,
            handler=self.SignalHandler(self)
        )
        self.sig_cli.connect()
        self.svrs.extend([
            rospy.Service(
                rosgraph.names.ns_join(self.id, name),
                srv_cls,
                handler
            )
            for name, srv_cls, handler in [
                ('calls', ros_webrtc.srv.ExampleGetCalls, self.on_calls),
                ('call_peer', ros_webrtc.srv.ExampleCallPeer, self.on_call_peer),
                ('hangup', ros_webrtc.srv.ExampleHangup, self.on_hangup),
            ]
        ])

    def calls(self):
        srv = rospy.ServiceProxy(
            rosgraph.names.ns_join(self.id, 'calls'),
            ros_webrtc.srv.ExampleGetCalls,
        )
        return srv().calls

    def on_calls(self, req):
        resp = ros_webrtc.srv.ExampleGetCallsResponse(calls=[
            ros_webrtc.msg.ExampleCall(id=pc.session_id, peer_id=pc.peer_id)
            for pc in self.pcs.values()
        ])
        return resp

    def call_peer(self, peer_id, session_id=None):
        srv = rospy.ServiceProxy(
            rosgraph.names.ns_join(self.id, 'call_peer'),
            ros_webrtc.srv.ExampleCallPeer
        )
        return srv(peer_id=peer_id, session_id=session_id)

    def on_call_peer(self, req):
        pc = self.create_pc(
            session_id=req.session_id or uuid.uuid4().hex,
            peer_id=req.peer_id,
            sdp_constraints=ros_webrtc.msg.MediaConstraints(
                mandatory=[],
                optional=[],
            ),
            video_sources=['*'],
            wait_for_service=30,
        )
        self.sig_cli.send('call', pc.peer_id, pc.session_id)
        pc.create_data_channel(
            label='uruoc',
            id=-1,
            reliable=False,
            ordered=False,
            protocol='application/vnd.mayfield.msg.v1+json; chunksize=32',
        )
        pc.create_data_channel(
            label='ruoc',
            id=-1,
            reliable=True,
            ordered=False,
            protocol='application/vnd.mayfield.msg.v1+json; chunksize=32',
        )
        pc.create_data_channel(
            label='roc',
            id=-1,
            reliable=True,
            ordered=True,
            protocol='application/vnd.mayfield.msg.v1+json; chunksize=32',
        )
        pc.create_data_channel(
            label='uruouc',
            id=-1,
            reliable=False,
            ordered=False,
            protocol='application/vnd.mayfield.msg.v1+json',
        )
        pc.create_data_channel(
            label='buruoc',
            id=-1,
            reliable=False,
            ordered=False,
            protocol='application/vnd.mayfield.msg.v1+json; chunksize=32',
        )
        pc.create_data_channel(
            label='broc',
            id=-1,
            reliable=True,
            ordered=True,
            protocol='application/vnd.mayfield.msg.v1+json; chunksize=32',
        )
        pc.create_data_channel(
            label='rosbridge',
            id=1,
            reliable=False,
            ordered=False,
            protocol='application/vnd.rosbridge.v1+json; chunksize=512',
        )
        pc.create_offer()
        call = ros_webrtc.msg.ExampleCall(id=pc.session_id, peer_id=pc.peer_id)
        resp = ros_webrtc.srv.ExampleCallPeerResponse(call=call)
        return resp

    def hangup(self, session_id, peer_id):
        srv = rospy.ServiceProxy(
            rosgraph.names.ns_join(self.id, 'hangup'),
            ros_webrtc.srv.ExampleHangup,
        )
        return srv(id=session_id, peer_id=peer_id)

    def on_hangup(self, req):
        pc = self.pcs.get((req.id, req.peer_id))
        if not pc:
            rospy.logwarn('no call w/ id="%s", peer_id="%s"', req.id, req.peer_id)
            return
        pc.delete()
        resp = ros_webrtc.srv.ExampleHangupResponse()
        return resp

    # Application

    def shutdown(self):
        super(Application, self).shutdown()
        self.sig_cli.close()

    def on_pc_delete(self, pc):
        if self.sig_cli.stream:
            self.sig_cli.send('hangup', pc.peer_id, pc.session_id)

    def on_pc_data_channel(self, pc, msg):
        return True

    def on_pc_ice_candidate(self, pc, msg):
        self.sig_cli.send('add_ice_candidate', pc.peer_id, pc.session_id, {
            'sdpMid': msg.sdp_mid,
            'sdpMLineIndex': msg.sdp_mline_index,
            'candidate': msg.candidate,
        })

    def on_pc_set_session_description(self, pc, msg):
        self.sig_cli.send('set_session_description', pc.peer_id, pc.session_id, {
            'type': msg.type,
            'sdp': msg.sdp
        })


def wait_until(predicate, timeout=10, period=1.0):
    expires_at = time.time() + timeout
    while not rospy.is_shutdown() and time.time() < expires_at:
        if predicate():
            return True
        time.sleep(period)
    return False


class TestPeerConnection(unittest.TestCase):

    def setUp(self):
        super(TestPeerConnection, self).setUp()
        signaling_url = 'ws://{0}:{1}'.format(
            rospy.get_param('signal_host'),
            rospy.get_param('signal_port'),
        )
        self.peer1 = Application(signaling_url)
        self.peer2 = Application(signaling_url)
        self.session_id = uuid.uuid4().hex
        self.timeout = 5.0
        self.get_host = rospy.ServiceProxy('get_host', ros_webrtc.srv.GetHost)

    def tearDown(self):
        self.peer1.shutdown()
        self.peer2.shutdown()
        rospy.sleep(rospy.Duration.from_sec(self.timeout))

    def test_call(self):
        # peer 1 call peer 2
        self.peer1.call_peer(peer_id=self.peer2.id, session_id=self.session_id)

        # wait for session establish
        rospy.sleep(rospy.Duration.from_sec(self.timeout))

        # now should have 2 peer connections for a single session
        self.get_host.wait_for_service(timeout=self.timeout)
        resp = self.get_host()
        pcs = resp.peer_connections
        self.assertEqual(len(pcs), 2)
        self.assertEqual(set(pc.session_id for pc in pcs), {self.session_id})
        self.assertEqual(
            set([pc.peer_id for pc in pcs]), {self.peer1.id, self.peer2.id}
        )

    def test_video(self):

        def _on_image(image, topic):
            frames[topic] += 1

        # peer 1 call peer 2
        self.peer1.call_peer(peer_id=self.peer2.id, session_id=self.session_id)

        # wait for setup
        rospy.sleep(self.timeout)

        # subscribe to pc images
        topic_fmt = 'session_{session}/peer_{peer}/video_{video}'
        frames = collections.Counter()
        subscribers = [
            rospy.Subscriber(
                topic_fmt.format(session=self.session_id, peer=peer_id, video=video),
                sensor_msgs.msg.Image,
                callback=_on_image,
                callback_args=topic_fmt.format(session=self.session_id, peer=peer_id, video=video)
            )
            for peer_id, video in [
                (self.peer1.id, 'downward'),
                (self.peer1.id, 'upward'),
                (self.peer2.id, 'downward'),
                (self.peer2.id, 'upward'),
            ]
        ]

        topics = [
            topic_fmt.format(session=self.session_id, peer=peer_id, video=video)
            for peer_id, video in [
                (self.peer1.id, 'downward'),
                (self.peer1.id, 'upward'),
                (self.peer2.id, 'downward'),
                (self.peer2.id, 'upward'),
            ]
        ]
        min_frames = 5
        self.assertTrue(wait_until(
            lambda: all(min_frames <= frames[topic] for topic in topics),
            360
        ))

    def test_data(self):

        def _send(peer, label, max_len):

            def _inner(event):
                buf = binascii.b2a_hex(os.urandom(random.randint(1, max_len)))
                peer.send_data(
                    self.session_id, label=label, encoding='ascii', buffer=buf,
                )
                send_len[label] += len(buf)

            return _inner

        def _recv(topic):

            def _inner(data):
                recv_len[label] += len(data.buffer)

            return _inner

        # peer 1 call peer 2
        self.peer1.call_peer(peer_id=self.peer2.id, session_id=self.session_id)

        # wait for setup
        rospy.sleep(rospy.Duration.from_sec(self.timeout))

        # send/recv data
        subs = []
        send_len, recv_len = collections.Counter(), collections.Counter()
        recv_topic_fmt = 'session_{session}/peer_{peer}/data_{data}'
        cases = [
            (self.peer1.id, self.peer2.id, 'uruoc', 2048),
            (self.peer2.id, self.peer1.id, 'uruoc', 2048),
            (self.peer1.id, self.peer2.id, 'ruoc', 2048),
            (self.peer2.id, self.peer1.id, 'ruoc', 2048),
            (self.peer1.id, self.peer2.id, 'roc', 2048),
            (self.peer2.id, self.peer1.id, 'roc', 2048),
            (self.peer1.id, self.peer2.id, 'uruouc', 128),
            (self.peer2.id, self.peer1.id, 'uruouc', 128),
        ]
        for send_peer, recv_peer, label, max_len in cases:
            # subscribe to dc
            recv_topic = recv_topic_fmt.format(
                session=self.session_id, peer=recv_peer, data=label,
            )
            subs.append(rospy.Subscriber(
                recv_topic, ros_webrtc.msg.Data, _recv(recv_topic),
            ))

            # send to dc
            _send(send_peer, label, max_len)

        # wait for data
        wait_until(
            lambda: all(
                send_len[label] == recv_len[label]
                for send_peer, recv_peer, label, _ in cases
            ), 60)

        # we recv'd what we sent
        for send_peer, recv_peer, label, _ in cases:
            self.assertEqual(send_len[label], recv_len[label])

    def test_reconnect(self):

        def _host_peer_cxns(nb):
            def _func():
                return nb == len(self.get_host().peer_connections)
            return  _func

        def _peer_calls(peer, nb):
            def _func():
                return nb == len(peer.calls())
            return  _func

        # peer 1 call peer 2
        self.peer1.call_peer(peer_id=self.peer2.id, session_id=self.session_id)

        # we have 2 peer connections
        self.assertTrue(wait_until(_host_peer_cxns(2), 30))
        pcs = self.get_host().peer_connections
        self.assertEqual(len(pcs), 2)
        self.assertEqual(set(pc.session_id for pc in pcs), {self.session_id})
        self.assertEqual(
            set([pc.peer_id for pc in pcs]), {self.peer1.id, self.peer2.id}
        )

        # peer 2 hangup on peer 1
        self.assertTrue(wait_until(_peer_calls(self.peer2, 1), 30))
        self.peer2.hangup(self.session_id, self.peer1.id)

        # we have 0 peer connections
        self.assertTrue(wait_until(_host_peer_cxns(0), 30))
        pcs = self.get_host().peer_connections
        self.assertEqual(len(pcs), 0)

        # peer 1 call peer 2 again
        self.peer1.call_peer(peer_id=self.peer2.id, session_id=self.session_id)

        # we have 2 peer connections again
        self.assertTrue(wait_until(_host_peer_cxns(2), 30))
        pcs = self.get_host().peer_connections
        self.assertEqual(len(pcs), 2)
        self.assertEqual(set(pc.session_id for pc in pcs), {self.session_id})
        self.assertEqual(
            set([pc.peer_id for pc in pcs]), {self.peer1.id, self.peer2.id}
        )

        # peer 2 hangup on peer 1
        self.assertTrue(wait_until(_peer_calls(self.peer2, 1), 30))
        self.peer2.hangup(self.session_id, self.peer1.id)

        # we have 0 peer connections
        self.assertTrue(wait_until(_host_peer_cxns(0), 30))
        pcs = self.get_host().peer_connections
        self.assertEqual(len(pcs), 0)

    def test_video_source_rotation(self):
        up_label = 'upward'
        down_label = 'downward'
        img_ars = {}
        ar = 4 / 3.

        rotate_video_source = rospy.ServiceProxy(
            'rotate_video_source', ros_webrtc.srv.RotateVideoSource
        )
        rotate_video_source.wait_for_service(self.timeout)

        def _on_image(image, topic):
            img_ars[topic] = image.width / (1. * image.height)

        # peer 1 call peer 2
        self.peer1.call_peer(peer_id=self.peer2.id, session_id=self.session_id)

        # wait for setup
        rospy.sleep(self.timeout)

        # image topics
        topic_fmt = 'session_{session}/peer_{peer}/video_{video}'
        up_topics = [
            topic_fmt.format(session=self.session_id, peer=peer_id, video=video)
            for peer_id, video in [
                (self.peer1.id, 'upward'),
                (self.peer2.id, 'upward'),
            ]
        ]
        down_topics = [
            topic_fmt.format(session=self.session_id, peer=peer_id, video=video)
            for peer_id, video in [
                (self.peer1.id, 'downward'),
                (self.peer2.id, 'downward'),
            ]
        ]

        # subscribe to pc images
        subscribers = [
            rospy.Subscriber(
                topic,
                sensor_msgs.msg.Image,
                callback=_on_image,
                callback_args=topic
            )
            for topic in up_topics + down_topics
        ]

        # wait for pc images
        self.assertTrue(wait_until(
            lambda: all(topic in img_ars for topic in up_topics + down_topics),
            60))

        # rotate up src and check aspect ratios
        for rotation, rotated_ar in [
                (0, ar),
                (90, 1 / ar),
                (180, ar),
                (270, 1 / ar),
                (0, ar),
                ]:
            rotate_video_source(up_label, rotation)

            self.assertTrue(wait_until(
                lambda: all(
                    topic in img_ars and img_ars[topic] == rotated_ar
                    for topic in up_topics
                ), 30))

            # no change
            self.assertTrue(all(
                topic in img_ars and img_ars[topic] == ar
                for topic in down_topics
            ))

        # rotate down src and check aspect ratios
        for rotation, rotated_ar in [
                (0, ar),
                (90, 1 / ar),
                (180, ar),
                (270, 1 / ar),
                (0, ar)]:
            rotate_video_source(down_label, rotation)

            self.assertTrue(wait_until(
                lambda: all(
                    topic in img_ars and img_ars[topic] == rotated_ar
                    for topic in down_topics
                ), 30))

            # no change
            self.assertTrue(all(
                topic in img_ars and img_ars[topic] == ar
                for topic in up_topics
            ))


def main():
    rospy.init_node(NAME)
    with ros_coverage():
        rostest.rosrun(PKG, NAME, TestPeerConnection)


if __name__ == '__main__':
    main()
