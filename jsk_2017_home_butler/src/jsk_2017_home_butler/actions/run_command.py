#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import copy
import rospy
from smach import State
from jsk_2017_home_butler.utilities import SpeechMixin
from jsk_2017_home_butler.utilities import snake_to_camel
from jsk_2017_home_butler.actions import PRIMITIVE_ACTIONS



class RunCommandAction(State, SpeechMixin):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'failed'] + PRIMITIVE_ACTIONS,
                       input_keys=['actions'],
                       output_keys=['actions', 'arguments'])

    def execute(self, userdata=None):
        actions = userdata.actions
        rospy.loginfo("%d actions left" % len(actions))
        if not actions:
            self.say("All commands are now done!")
            return 'succeeded'

        next_action = actions.pop(0)

        # update userdata
        userdata.actions = actions
        userdata.arguments = next_action.arguments

        return snake_to_camel(next_action.type)


if __name__ == '__main__':
    from smach import StateMachine

    rospy.init_node("run_command")

    ac = RunCommandAction()

    sm = StateMachine(outcomes=['succeeded', 'failed'])
    with sm:
        StateMachine.add("run_command", ac,
                         transitions={'succeeded': 'succeeded',
                                      'failed': 'failed'})

    result = sm.execute()

    print "result:", result, "arguments:", sm.userdata.arguments
