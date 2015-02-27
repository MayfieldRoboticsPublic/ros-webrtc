#!/usr/bin/env python
import base64
import collections
import json
import random
import time
import unittest
import uuid

import rospy
import rostest
import sensor_msgs.msg
from ws4py.client.threadedclient import WebSocketClient

import ros_webrtc.msg
import ros_webrtc.srv
from pprint import pprint


PKG = 'ros_webrtc'
NAME = 'session_test'


class TestSession(unittest.TestCase):
    
    def setUp(self):
        super(TestSession, self).setUp()
        self.srv_timeout = 2.0
        self.signal_timeout = 2.0
        self.peer1_id = rospy.get_param('~peer1_id')
        self.peer2_id = rospy.get_param('~peer2_id')
        signaling_url = rospy.get_param('~signaling_url')
        rospy.loginfo('connecting to signaling @ "%s"', signaling_url)
        self.signaling = Signaling(signaling_url)
        self.signaling.connect()
        self.addCleanup(self.signaling.close)
    
    @unittest.skip('')
    def test_setup(self):
        session = uuid.uuid4().hex
        self.signaling.publish(self.peer1_id, session, 'start', {'peer': self.peer2_id})
        time.sleep(self.signal_timeout)
 
        srv = rospy.ServiceProxy('/host/get_host', ros_webrtc.srv.GetHost)
        srv.wait_for_service(timeout=self.srv_timeout)
        resp = srv()
        self.assertEqual(len(resp.sessions), 2)
        self.assertItemsEqual(
            [s.id for s in resp.sessions], [session, session]
        )
        self.assertItemsEqual(
            [s.peer_id for s in resp.sessions], [self.peer1_id, self.peer2_id]
        )
    
    @unittest.skip('')
    def test_video(self):
        session = uuid.uuid4().hex
        self.signaling.publish(self.peer1_id, session, 'start', {'peer': self.peer2_id})
        time.sleep(self.signal_timeout)
        
        frames = collections.Counter()
        
        def _on_image(image, topic):
            frames[topic] += 1
        
        topic_fmt = '/session_{session}/peer_{peer}/video_{video}'
        
        subscribers = [
            rospy.Subscriber(
                topic_fmt.format(session=session, peer=peer, video=video),
                sensor_msgs.msg.Image,
                callback=_on_image,
                callback_args=topic_fmt.format(session=session, peer=peer, video=video)
            )
            for peer, video in [
                (self.peer1_id, 'downward'),
                (self.peer1_id, 'upward'),
                (self.peer2_id, 'downward'),
                (self.peer2_id, 'upward'),
            ]
        ]
        
        shutdown = rospy.Timer(
            rospy.Duration(20.0),
            lambda event: rospy.signal_shutdown('done'),
            oneshot=True
        )
        
        rospy.spin()
        
        for topic in frames.iterkeys():
            self.assertTrue(10 <= frames[topic] <= 100)
        
    def test_data(self):
        session = uuid.uuid4().hex
        self.signaling.publish(self.peer1_id, session, 'start', {'peer': self.peer2_id})
        time.sleep(self.signal_timeout)
        
        sent = collections.Counter()
        
        def _send(publisher, label, max_len):
            
            def _inner(event):
                buffer = base64.b64encode(''.join(
                    chr(random.randint(0, 255)) for _ in range(random.randint(1, max_len))
                ))
                publisher.publish(label=label, encoding='utf-8', buffer=buffer)
                sent[publisher.name] += len(buffer)
                
            return _inner
            
        recvd = collections.Counter()
        
        def _recv(topic):
            
            def _inner(data):
                recvd[topic] += len(data.buffer)
                
            return _inner
        
        topic_fmt = '/session_{session}/peer_{peer}/data_{data}'
        sends = []
        recvs = []
        
        cases = [
            (self.peer1_id, self.peer2_id, 'uruoc', 2048),
            (self.peer2_id, self.peer1_id, 'uruoc', 2048),
            (self.peer1_id, self.peer2_id, 'ruoc', 2048),
            (self.peer2_id, self.peer1_id, 'ruoc', 2048),
            (self.peer1_id, self.peer2_id, 'roc', 2048),
            (self.peer2_id, self.peer1_id, 'roc', 2048),
            (self.peer1_id, self.peer2_id, 'uruouc', 128),
            (self.peer2_id, self.peer1_id, 'uruouc', 128),
        ]
        
        for send_peer, recv_peer, label, max_len in cases:
            # send
            publisher = rospy.Publisher(
                topic_fmt.format(session=session, peer=send_peer, data=label) + '/send',
                ros_webrtc.msg.Data,
                queue_size=1,
            )
            sends.append(rospy.Timer(
                rospy.Duration(1.0),
                _send(publisher, label, max_len),
            ))
            
            # recv
            recvs.append(rospy.Subscriber(
                topic_fmt.format(session=session, peer=recv_peer, data=label) + '/recv',
                ros_webrtc.msg.Data, 
                _recv(topic_fmt.format(session=session, peer=recv_peer, data=label) + '/recv'),
            ))
        
        stop_sending = rospy.Timer(
            rospy.Duration(2.0),
            lambda event: [timer.shutdown() for timer in sends],
            oneshot=True
        )
        
        shutdown = rospy.Timer(
            rospy.Duration(10.0),
            lambda event: rospy.signal_shutdown('done'),
            oneshot=True
        )
        
        rospy.spin()
        
        for send_peer, recv_peer, label, _ in cases:
            send_topic = topic_fmt.format(session=session, peer=send_peer, data=label) + '/send'
            recv_topic = topic_fmt.format(session=session, peer=recv_peer, data=label) + '/recv'
            self.assertEqual(sent[send_topic], recvd[recv_topic])


class Signaling(WebSocketClient):
    
    def __init__(self, url):
        super(Signaling, self).__init__(
            url, headers=[('X-Channel', uuid.uuid4().hex)]
        )
        
    def publish(self, channel, session, type, payload=None, callback=None):
        message = {
            'channel': channel,
            'session': session,
            'type': type,
            'payload': payload,
            'callback': callback,
        }
        return self.send(json.dumps(message), binary=False)


def main():
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestSession)


if __name__ == '__main__':
    main()
