#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>


from jsk_2017_home_butler.utils import camel_to_snake

PRIMITIVE_ACTIONS = [
    camel_to_snake('AnswerQuestion'),
    camel_to_snake('FindObject'),
    camel_to_snake('FindPerson'),
    camel_to_snake('Deliver'),
    camel_to_snake('GoTo'),
    camel_to_snake('PickObject'),
    camel_to_snake('PlaceObject'),
    camel_to_snake('Speak'),
]


from listen_command import ListenCommandAction
from wait_for_command import WaitForCommandAction
from run_command import RunCommandAction
