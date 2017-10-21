#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>


PRIMITIVE_ACTIONS = [
    'AnswerQuestion',
    'FindObject',
    'FindPerson',
    'Deliver',
    'Go',
    'Pick',
    'Place',
    'Speak',
]


from listen_command import ListenCommandAction
from wait_for_command import WaitForCommandAction
from run_command import RunCommandAction
from recover import RecoverAction
