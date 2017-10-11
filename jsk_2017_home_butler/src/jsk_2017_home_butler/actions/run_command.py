#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
from smach import State
from jsk_2017_home_butler.utils import SpeechMixin
from jsk_2017_home_butler.actions import PRIMITIVE_ACTIONS



class RunCommandAction(State, SpeechMixin):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'failed'] + PRIMITIVE_ACTIONS,
                       input_keys=['commands'],
                       output_keys=['arguments'])

    def execute(self, userdata=None):
        return 'succeeded'


if __name__ == '__main__':
    pass
