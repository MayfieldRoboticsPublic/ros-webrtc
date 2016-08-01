#!/usr/bin/env python
import collections
import time
import unittest

import rospy
import rostest

import sensor_msgs.msg

import ros_webrtc.srv

from ros_coverage import ros_coverage


PKG = 'ros_webrtc'
NAME = 'host_test'


class TestHost(unittest.TestCase):

    def _video_topic(self, label):
        return 'local/{label}'.format(label=label)

    def _audio_topic(self, label):
        return 'local/{label}'.format(label=label)

    def _service(self, name):
        return '{0}'.format(name)

    def test_video(self):

        def callback(image, topic):
            frames[topic] += 1

        frames = collections.Counter();

        topics = [
            self._video_topic('downward'),
            self._video_topic('upward'),
        ]

        subscribers = [
            rospy.Subscriber(topic, sensor_msgs.msg.Image, callback, topic)
            for topic in topics
        ]

        min_frames, expires_at = 3, time.time() + 60

        while (not rospy.core.is_shutdown() and
               not all(min_frames <= frames[topic] for topic in topics) and
               time.time() < expires_at):
            rospy.sleep(2.0)

        for topic in topics:
            self.assertGreaterEqual(frames[topic], min_frames)

    def test_conf(self):
        srv = rospy.ServiceProxy(self._service('get_host'), ros_webrtc.srv.GetHost)
        srv.wait_for_service(5.0)
        resp = srv()
        self.assertEqual(len(resp.sdp_constraints.optional), 1)
        self.assertTrue(resp.sdp_constraints.optional[0].key == 'DtlsSrtpKeyAgreement')
        self.assertTrue(resp.sdp_constraints.optional[0].value)
        self.assertEqual(len(resp.sdp_constraints.mandatory), 0)
        self.assertEqual(len(resp.video_sources), 2)
        self.assertEqual(len(resp.audio_sources), 1)
        self.assertEqual(resp.peer_connections, [])


def main():
    rospy.init_node(NAME)
    with ros_coverage():
        rostest.rosrun(PKG, NAME, TestHost)


if __name__ == '__main__':
    main()
