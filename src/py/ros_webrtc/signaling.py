"""
"""
import json

import rospy

from ws4py.client.threadedclient import WebSocketClient


class SignalClient(WebSocketClient):

    def __init__(self, *args, **kwargs):
        self.handler = kwargs.pop('handler')
        super(SignalClient, self).__init__(*args, **kwargs)
        self._map = {
            'call': self.handler.on_call,
            'pickup': self.handler.on_pickup,
            'add_ice_candidate': self.handler.on_add_ice_candidate,
            'set_session_description': self.handler.on_set_session_description,
            'hangup': self.handler.on_hangup,
        }

    # WebSocketClient

    def send(self, type_, channel, session, payload=None):
        msg = {
            'channel': channel,
            'session': session,
            'type': type_,
            'payload': payload,
        }
        text = json.dumps(msg, indent=4)
        rospy.logdebug('sending signal\n%s', text)
        return super(SignalClient, self).send(text, binary=False)

    def received_message(self, message):
        msg = json.loads(str(message))
        channel, session, type_, payload = (
            msg['channel'],
            msg['session'],
            msg['type'],
            msg.get('payload'),
        )
        rospy.logdebug(
            'received signal - channel="%s", session="%s", type="%s" payload=\n%s',
            channel, session, type_, json.dumps(payload, indent=4)
        )
        try:
            handler = self._map.get(type_)
            if handler is None:
                rospy.logwarn('signal type "%s" not supported', type_)
                return
            try:
                handler(channel, session, payload)
            except DropSignal, ex:
                rospy.loginfo(
                    'dropping signal - channel="%s", session="%s", type="%s" - %s',
                    channel, session, type_, ex,
                )
        except Exception, ex:
            rospy.logerr(ex)


class DropSignal(Exception):

    pass


class SignalHandler(object):

    def on_call(self, channel, session, payload):
        pass

    def on_pickup(self, channel, session, payload):
        pass

    def on_add_ice_candidate(self, channel, session, payload):
        pass

    def on_set_session_description(self, channel, session, payload):
        pass

    def on_hangup(self, channel, session, payload):
        pass
