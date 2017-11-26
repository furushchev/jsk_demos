#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import re
import actionlib
import rospy
from jsk_gui_msgs.msg import VoiceMessage
from sound_play.msg import SoundRequest
from sound_play.msg import SoundRequestAction, SoundRequestGoal
from speech_recognition_msgs.srv import SpeechRecognition
from jsk_2017_home_butler.classifier import WordClassifier


def snake_to_camel(text):
    return str().join(s.title() for s in text.split('_'))


def camel_to_snake(text):
    return (text[0] + re.sub('([A-Z])', lambda s: '_' + s.group(1), text[1:])).lower()


_ACTION_CLIENTS = {}
class SpeechMixin(object):
    def say(self, text, lang="", wait=True, timeout=10, ns=None):
        global _ACTION_CLIENTS
        msg = SoundRequest(
            sound=SoundRequest.SAY,
            command=SoundRequest.PLAY_ONCE,
            arg=text,
            arg2=lang)

        if ns is None:
            if lang == "ja":
                ns = "robotsound_jp"
            else:
                ns = "robotsound"
        if ns not in _ACTION_CLIENTS:
            _ACTION_CLIENTS[ns] = actionlib.SimpleActionClient(ns, SoundRequestAction)
        ac = _ACTION_CLIENTS[ns]
        if not ac.wait_for_server(rospy.Duration(1)):
            rospy.logwarn("Action server /%s not found." % ns)
            del _ACTION_CLIENTS[ns]

            pub = rospy.Publisher(ns, SoundRequest, queue_size=1)
            if pub.get_num_connections() == 0:
                rospy.sleep(1)
            pub.publish(msg)
            sleep_length = len(msg.arg) * 0.1 + 0.2
            if wait:
                rospy.sleep(sleep_length)
            if pub.get_num_connections() == 0:
                rospy.loginfo("Robot said: %s" % msg.arg)
        else:
            goal = SoundRequestGoal(sound_request=msg)
            ac.send_goal(goal)
            if wait:
                ac.wait_for_result(timeout=rospy.Duration(timeout))
        return True

    def _voice_callback(self, msg):
        self.speech_msg = msg.texts

    def listen_topic(self, duration=3.0, retry=2, choices=None):
        self.speech_msg = None
        sub = rospy.Subscriber("/Tablet/voice", VoiceMessage,
                               self._voice_callback, queue_size=1)
        rate = rospy.Rate(10)
        speech = None
        for i in range(int(duration*10)):
            speech = self.speech_msg
            if speech:
                if choices:
                    for t in speech:
                        if t in choices:
                            speech = t
                            break
                else:
                    speech = speech[0]
                    break
            rate.sleep()
        sub.unregister()
        self.speech_msg = None
        return speech

    def listen(self, duration=3.0, retry=2, grammar=None, threshold=0.9, choices=None, quiet=False, ns=None):
        """Listen to speech.
        Either choices or grammar must be specified.

        Args:
            choices ([str], default: None):
                Candidate words to be recognized
                Enabled only for julius engine with isolated word recognition
            duration (float, default: 3.0):
                Maximum duration to listen
            quiet (bool, default: False):
                Sounds on start and end of listening if true
            retry (int, default: 2):
                Number of retry to listen
            grammar (str, default: None):
                Grammar name to be recognized
                Enabled only for julius engine with grammar recognition
            threshold (float, default: None):
                Threshold for speech recognition.
            ns (string, default: None):
                namespace for speech-to-text service
        Returns:
            Speech sentence if successfully recognized.
            Returns None otherwise.
        Example:
            choice = self.listen(choices=['はい', 'いいえ'])
            or
            sentence = self.listen(grammar='gpsr_en')
        """

        ns = ns or "speech_recognition"

        try:
            rospy.wait_for_service(ns, timeout=1)
        except rospy.ROSException as e:
            rospy.logerr("service /%s not yet advertised?" % ns)
            return self.listen_topic(duration=duration, retry=retry, choices=choices)

        sr = rospy.ServiceProxy(ns, SpeechRecognition)

        while retry >= 0:
            if rospy.is_shutdown():
                return ''
            retry -= 1
            res = sr(duration=duration,
                     quiet=quiet,
                     threshold=threshold)
            if res.result.transcript:
                return res.result.transcript[0]

        rospy.logerr("Could not recognize speech")
        return ''

    def ask(self, query, retry=2, **kwargs):
        """Ask question by speaking and get answer by listening

        Args:
            query (string):
                Question
            For other arguments, see method `listen`.
        Returns:
            Answer sentence if successfully recognized.
            Returns None otherwise.
        """
        while retry >= 0:
            if rospy.is_shutdown():
                return ''
            retry -= 1
            self.say(query, wait=True)
            rospy.sleep(0.5)
            answer = self.listen(retry=0, **kwargs)
            if answer:
                return answer

        rospy.logerr("Failed to get answer for the query")
        return ''


if __name__ == '__main__':
    rospy.init_node("utils")
    m = SpeechMixin()
    print "answer:", m.ask("Test")
