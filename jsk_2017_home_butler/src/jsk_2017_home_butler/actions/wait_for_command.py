#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
from smach import State


class WaitForCommandAction(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'failed'])

    def execute(self, userdata=None):
        rospy.sleep(20)  # FIXME: implement active attention estimation
        return 'succeeded'


if __name__ == '__main__':
    from smach import StateMachine

    rospy.init_node("wait_for_command")

    ac = WaitForCommandAction()

    sm = StateMachine(outcomes=['succeeded', 'failed'])
    with sm:
        StateMachine.add("wait_for_command", ac,
                         transitions={'succeeded': 'succeeded',
                                      'failed': 'failed'})
    result = sm.execute()

    print "result:", result

