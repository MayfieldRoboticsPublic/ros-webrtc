#!/usr/bin/env python
import logging
import os
import socket
import subprocess
import sys
import threading
import time
import unittest
import urlparse
import uuid

import cv2
import numpy
import ros_webrtc.msg
import ros_webrtc.srv
import rospy
import rostest
from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.support.ui import WebDriverWait
from xvfbwrapper import Xvfb

from ros_coverage import ros_coverage


PKG = 'ros_webrtc'
NAME = 'site_test'


class SettingsMixin(object):

    @property
    def settings(self):
        return self.driver.execute_script("""\
var o = {};
settings.pairs().forEach(function (kv) {
    o[kv[0]] = kv[1];
});
return o;
""")

    @settings.setter
    def settings(self, value):
        return self.driver.execute_script("""\
settings.set(arguments[0]);
settings.save();
""",
            value
        )


def wait_until(predicate, timeout=10, period=1.0):
    expires_at = time.time() + timeout
    while not rospy.is_shutdown() and time.time() < expires_at:
        if predicate():
            return True
        time.sleep(period)
    return False


class Page(SettingsMixin):

    def __init__(self, url, default_timeout=30):
        self.url = url
        self.default_timeout = default_timeout
        self.driver = None

    @property
    def peer_id(self):
        return urlparse.urlparse(self.url)[5].lstrip('#')

    def load(self, driver, timeout=15, poll_frequency=2):
        result = driver.get(self.url)
        self.driver = driver
        try:
            self.wait_until(
                lambda driver: self.is_loaded, timeout, poll_frequency
            )
        except:
            self.unload()
            raise
        return result

    def reload(self, timeout=15, poll_frequency=2):
        result = self.driver.refresh();
        try:
            result = self.wait_until(
                lambda driver: self.is_loaded, timeout, poll_frequency
            )
        except:
            self.unload()
            raise
        return result

    @property
    def is_loaded(self):
        return self.driver.execute_script("""\
return (
    typeof signaling !== 'undefined'
);
""")

    def unload(self):
        self.driver = None

    def wait_until(self, predicate, timeout=30, poll_frequency=2):
        WebDriverWait(
            self.driver, timeout, poll_frequency=poll_frequency
        ).until(lambda driver: predicate(driver))

    @property
    def is_connected_to_signaling(self):
        return self.driver.execute_script("""\
return (
    typeof signaling !== 'undefined' &&
    typeof signaling.ws !== 'undefined' &&
    signaling.ws.readyState === 1
);
""")

    @property
    def is_connected_to_peer(self):
        return self.driver.execute_script("""\
return (
    typeof call !== 'undefined' &&
    typeof call.pc !== 'undefined' &&
    (call.pc.iceConnectionState === 'completed' ||
     call.pc.iceConnectionState === 'connected')
);
""")

    @property
    def video(self):
        return Video.by_css_selector(self.driver, 'video')

    def send_message(self, msg):
        e = self.driver.find_element_by_css_selector('#msg-send input')
        e.send_keys(msg)
        e.send_keys(Keys.ENTER)

    def received_messages(self):
        return self.driver.execute_script("""\
var msgs = [];
$('#msg-recv-log ol li').each(function () {
    msgs.push($(this).text());
});
return msgs;
""")


class Element(object):

    @classmethod
    def by_xpath(cls, driver, *args, **kwargs):
        return cls(
            driver.find_element_by_xpath(*args, **kwargs), driver
        )

    @classmethod
    def by_css_selector(cls, driver, *args, **kwargs):
        return cls(
            driver.find_element_by_css_selector(*args, **kwargs), driver
        )

    @classmethod
    def by_id(cls, driver, *args, **kwargs):
        return cls(
            driver.find_element_by_id(*args, **kwargs), driver
        )

    def __init__(self, element, driver):
        self.element = element
        self.driver = driver

    def wait_until(self, predicate, timeout=60, poll_frequency=2):
        WebDriverWait(
            self.driver,
            timeout,
            poll_frequency=poll_frequency
        ).until(lambda driver: predicate)

    @property
    def bounding_client_rect(self):
        return self.driver.execute_script("""\
var elm = arguments[0];
return elm.getBoundingClientRect();
""",
            self.element,
        )

    def scroll_to(self, delay=0.1):
        self.driver.execute_script("""\
window.scrollTo(arguments[0], arguments[1]);
""",
            self.element.location['x'],
            self.element.location['y']
        )
        time.sleep(delay)

    def screenshot(self):
        self.scroll_to()
        left, top = self.driver.execute_script("""\
return [window.scrollX, window.scrollY];
""",
        )
        png = self.driver.get_screenshot_as_png()
        im = cv2.imdecode(
            numpy.fromstring(png, numpy.uint8), cv2.CV_LOAD_IMAGE_COLOR
        )
        y = self.element.location['y'] - top
        x = self.element.location['x'] - left
        return im[
            max(0, y):y + self.element.size['height'],
            max(0, x):x + self.element.size['width']
        ]


class Video(Element):

    loaded_threshold = 0.75

    empty = (255, 255, 255)

    @property
    def is_loaded(self):
        # https://developer.mozilla.org/en-US/docs/Web/API/HTMLMediaElement/readyState
        return self.element.get_attribute('readyState') == '4'


def browser_log(driver):
    loggers = {}
    levels = {
        'DEBUG': logging.DEBUG,
        'INFO': logging.INFO,
        'WARNING': logging.WARN,
        'ERROR': logging.ERROR,
        'SEVERE': logging.ERROR,
    }
    for entry in driver.get_log('browser'):
        src = entry.get('source')
        if not src:
            continue
        logger = loggers.get(src)
        if not logger:
            logger = loggers[src] = logging.getLogger(src)
        logger.log(levels[entry['level']], entry['message'])


def config_logging(log_level=logging.DEBUG):
    logging.basicConfig(
        format='%(levelname)s : %(name)s : %(message)s',
        level=log_level,
        stream=sys.stderr,
    )


class TestSite(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        config_logging()
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.bind(('', 0))
        cls.site_port = sock.getsockname()[1]
        sock.close()
        cls.server = subprocess.Popen(
            args=['python', '-m', 'SimpleHTTPServer', str(cls.site_port)],
            cwd=os.path.join(rospy.get_param('~dist'))
        )
        if bool(rospy.get_param('~headless', False)):
            cls.xvfb = Xvfb()
            cls.xvfb.start()
        else:
            cls.xvfb = None

    @classmethod
    def tearDownClass(cls):
        cls.server.terminate()
        cls.server = None
        cls.site_port = None
        if cls.xvfb:
            cls.xvfb.stop()
            cls.xvfb = None

    def setUp(self):
        desired_caps = webdriver.DesiredCapabilities.CHROME.copy()
        desired_caps.update({
            'loggingPrefs': {
                'browser': 'ALL',
                'driver': 'ALL',
            },
        })
        self.driver = webdriver.Chrome(
            desired_capabilities=desired_caps,
        )
        self.my_id = uuid.uuid4().hex
        self.peer_id = rospy.get_param('peer_id')
        self.signaling_url = 'ws://{0}:{1}'.format(
            rospy.get_param('signal_host'),
            rospy.get_param('signal_port'),
        )

    def tearDown(self):
        browser_log(self.driver)

    def test_peer_connection(self):
        self.page.wait_until(lambda _: self.page.is_connected_to_peer)

    def test_peer_hangup(self):
        self.page.wait_until(lambda _: self.page.is_connected_to_peer)
        self.page.video.wait_until(lambda _: self.page.video.is_loaded)

        get_host = rospy.ServiceProxy(
            'get_host', ros_webrtc.srv.GetHost
        )
        get_host.wait_for_service(10.0)

        calls = rospy.ServiceProxy(
            'example/calls', ros_webrtc.srv.ExampleGetCalls
        )
        calls.wait_for_service(10.0)
        cs = [c for c in calls().calls if c.peer_id == self.my_id]
        self.assertGreaterEqual(len(cs), 1)
        c = cs[0]
        pcs = [pc for pc in get_host().peer_connections if pc.session_id == c.id]
        self.assertGreaterEqual(len(pcs), 1)

        closed = threading.Event()
        on_close = rospy.Subscriber(
            'close', ros_webrtc.msg.Close, callback=lambda _: closed.set(),
        )
        self.assertTrue(wait_until(lambda: on_close.get_num_connections() > 0))

        hangup = rospy.ServiceProxy(
            'example/hangup', ros_webrtc.srv.ExampleHangup
        )
        hangup.wait_for_service(10.0)
        self.assertIsNotNone(hangup(id=c.id, peer_id=c.peer_id))

        self.assertTrue(closed.wait(10.0))

        self.assertFalse(any(c.peer_id == self.my_id for c in calls().calls))
        self.assertTrue(wait_until(lambda: (
            len([pc for pc in get_host().peer_connections if pc.session_id == c.id]) == 0
        )))
        pcs = [pc for pc in get_host().peer_connections if pc.session_id == c.id]
        self.assertEqual(len(pcs), 0)

    def test_video(self):
        self.page.wait_until(lambda _: self.page.is_connected_to_peer)
        self.page.video.wait_until(lambda _: self.page.video.is_loaded)
        base_im = None
        expires_at = time.time() + 60
        threshold = 0.01
        changes = 0
        min_changes = 3
        while (not rospy.is_shutdown() and
               changes < min_changes and
               time.time() < expires_at):
            im = self.page.video.screenshot()
            if base_im is None:
                base_im = im
            elif im.shape != base_im.shape:
                base_im = im
            else:
                diff = cv2.norm(im, base_im, cv2.NORM_L2) / float(im.size)
                if diff > threshold:
                    changes += 1
                    base_im = im
            rospy.sleep(0.5)
        self.assertGreaterEqual(changes, min_changes)

    def test_data(self):
        self.page.wait_until(lambda _: self.page.is_connected_to_peer)
        self.assertEqual(self.page.received_messages(), [])

        self.page.send_message('quack')
        self.assertTrue(wait_until(
            lambda: 'quack' in self.page.received_messages(),
            timeout=60
        ))
        self.assertEqual(self.page.received_messages(), ['quack'])

        self.page.send_message('moo')
        self.assertTrue(wait_until(
            lambda: 'moo' in self.page.received_messages(), timeout=60
        ))
        self.assertEqual(self.page.received_messages(), ['quack', 'moo'])


class TestSiteWait(TestSite):

    def setUp(self):
        super(TestSiteWait, self).setUp()
        url = 'http://localhost:{0}/#{1}/wait'.format(
            self.site_port, self.my_id,
        )
        self.page = Page(url)
        self.page.load(self.driver)
        self.page.settings = {
            'signaling_server': {
                'url': self.signaling_url,
            },
        }
        self.page.reload()

        # connect to signaling
        self.page.wait_until(lambda _: self.page.is_connected_to_signaling)

        # peer calls me
        call_peer = rospy.ServiceProxy(
            'example/call',
            ros_webrtc.srv.ExampleCallPeer
        )
        call_peer.wait_for_service(10.0)
        call_peer(peer_id=self.my_id, session_id=None)


class TestSiteCall(TestSite):

    def setUp(self):
        super(TestSiteCall, self).setUp()
        url = 'http://localhost:{0}/#{1}/call/{2}'.format(
            self.site_port, self.my_id, self.peer_id,
        )
        self.page = Page(url)
        self.page.load(self.driver)
        self.page.settings = {
            'signaling_server': {
                'url': self.signaling_url,
            },
        }
        self.page.reload()

    def test_signaling(self):
        self.page.wait_until(lambda _: self.page.is_connected_to_signaling)


def main():
    rospy.init_node(NAME)

    if len(sys.argv) == 1:
        test_cls = TestSiteCall
    else:
        if sys.argv[1] == 'call':
            test_cls = TestSiteCall
        elif sys.argv[1] == 'wait':
            test_cls = TestSiteWait
        else:
            raise ValueError('Invalid test "{0}".'.format(sys.argv[1]))

    with ros_coverage():
        rostest.rosrun(PKG, NAME, test_cls)


if __name__ == '__main__':
    main()
