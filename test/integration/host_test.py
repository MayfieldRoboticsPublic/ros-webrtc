#!/usr/bin/env python
import collections
import unittest

import rospy
import rostest

import sensor_msgs.msg

import ros_webrtc.srv


PKG = 'ros_webrtc'
NAME = 'host_test'


class TestHost(unittest.TestCase):
    
    def _video_topic(self, host, label):
        return '/{host}/local/video_{label}'.format(host=host, label=label)
    
    def _audio_topic(self, host, label):
        return '/{host}/local/audio_{label}'.format(host=host, label=label)
    
    def _service(self, host, name):
        return '/{host}/{name}'.format(host=host, name=name)

    @unittest.skip("for now")
    def test_video(self):
        
        def callback(image, topic):
            frames[topic] += 1
            
        frames = collections.Counter();
        
        topics = [
            self._video_topic('host', 'downward'),
            self._video_topic('host', 'upward'),
        ]
        
        subscribers = [
            rospy.Subscriber(topic, sensor_msgs.msg.Image, callback, topic)
            for topic in topics
        ]

        shutdown = rospy.Timer(
            rospy.Duration(20.0),
            lambda event: rospy.signal_shutdown('done'),
            oneshot=True
        )
        
        rospy.spin()

        for topic in topics:
            self.assertTrue(10 <= frames[topic] <= 100)
    
    def test_conf(self):
        srv = rospy.ServiceProxy(self._service('host', 'get_host'), ros_webrtc.srv.GetHost)
        resp = srv()
        self.assertEqual(len(resp.sdp_constraints.optional), 1)
        self.assertTrue(resp.sdp_constraints.optional[0].key == 'DtlsSrtpKeyAgreement')
        self.assertTrue(resp.sdp_constraints.optional[0].value)
        self.assertEqual(len(resp.sdp_constraints.mandatory), 0)
        self.assertEqual(len(resp.tracks), 3)
        self.assertDictEqual(
            collections.Counter(track.kind for track in resp.tracks), 
            {'video': 2, 'audio': 1}
        )
        self.assertEqual(resp.sessions, []) 


def main():
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestHost)


if __name__ == '__main__':
    main()
