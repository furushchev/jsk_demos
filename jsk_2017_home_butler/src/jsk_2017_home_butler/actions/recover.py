#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
from smach import State


class RecoverAction(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded'])

    def execute(self, userdata=None):
        rospy.logwarn("recover action")
        return 'succeeded'


if __name__ == '__main__':
    from smach import StateMachine

    rospy.init_node("recover")

    ac = RecoverAction()

    sm = StateMachine(outcomes=['succeeded'])
    with sm:
        StateMachine.add("Recover", ac,
                         transitions={'succeeded': 'succeeded'})

    result = sm.execute()

    print "result:", result

