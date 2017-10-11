#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
from smach import State
from jsk_2017_home_butler.utils import SpeechMixin


class ListenCommandAction(State, SpeechMixin):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'failed'],
                       output_keys=['query'])

    def execute(self, userdata=None):
        self.say("Hi!")
        rospy.sleep(0.3)
        answer = ""
        for i in range(3):
            if i == 0:
                query = "May I help you?"
            else:
                query = "Please say again?"
            answer = self.ask(query, duration=15.0, threshold=0.6)
            if answer:
                break
            self.say("Sorry, I cannot get you.")

        if not answer:
            return 'failed'

        self.say("You said: %s" % answer)
        userdata.query = answer

        # TODO: recognize query

        return 'succeeded'


if __name__ == '__main__':
    from smach import StateMachine

    rospy.init_node("listen_command")

    ac = ListenCommandAction()

    sm = StateMachine(outcomes=['succeeded', 'failed'])
    with sm:
        StateMachine.add("listen_command", ac,
                         transitions={'succeeded': 'succeeded',
                                      'failed': 'failed'},
                         remapping={'query': 'query'})
    result = sm.execute()

    print "result:", result, "userdata.query:", sm.userdata.query
