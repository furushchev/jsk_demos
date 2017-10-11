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


if __name__ == '__main__':
    pass
