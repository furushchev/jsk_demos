#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import re
import rospy
from smach import StateMachine
from smach_ros import SimpleActionState

from actionlib_tutorials.msg import FibonacciAction
from jsk_2017_home_butler.msg import ListenCommandAction
from jsk_2017_home_butler.msg import AnswerQuestionAction
from jsk_2017_home_butler.msg import FindObjectAction
from jsk_2017_home_butler.msg import FindPersonAction
from jsk_2017_home_butler.msg import DeliverAction
from jsk_2017_home_butler.msg import GoToAction
from jsk_2017_home_butler.msg import PickObjectAction
from jsk_2017_home_butler.msg import PlaceObjectAction
from jsk_2017_home_butler.msg import SpeakAction


def hoge():
    StateMachine.add(
        'fib',
        SimpleActionState('fibonacci', FibonacciAction,
                          goal_slots=['order'],
                          result_slots=['sequence']),
        transitions={'succeeded': 'succeeded',
                     'aborted': 'failed',
                     'preempted': 'failed'},
        remapping={'order': 'hoge',
                   'sequence': 'fuga'})

def test_smach():
    sm = StateMachine(['succeeded', 'failed'])
    with sm:
        hoge()
    return sm

def snake_to_camel(text):
    return str().join(s.title() for s in text.split('_'))

def make_action(name, spec,
                succeeded='succeeded', failed='failed',
                goal={}, result={}):
    remapping = goal.copy()
    remapping.update(result)
    StateMachine.add(
        snake_to_camel(name),
        SimpleActionState(name, spec,
                          goal_slots=goal.keys(),
                          result_slots=result.keys()),
        transitions={'succeeded': succeeded,
                     'aborted': failed,
                     'preempted': failed},
        remapping=remapping)

def make_state_machine():
    sm = StateMachine(['succeeded', 'failed'])
    with sm:
        make_action('wait_for_command', WaitForCommandAction,
                    succeeded='listen_command', failed='wait_for_command')
        make_action('listen_command', ListenCommandAction,
                    succeeded='run_command', failed='wait_for_command',
                    result={'task': 'commands'})
        # TODO: run_command
        # TODO: recover
        make_action('find_person', FindPersonAction,
                    succeeded='run_command', failed='recover')
        make_action('speak', SpeakAction,
                    succeeded='run_command', failed='recover')
        make_action('go_to', GoToAction,
                    succeeded='run_command', failed='recover')
        make_action('find_object', FindObjectAction,
                    succeeded='run_command', failed='recover')
        make_action('answer_question', AnswerQuestionAction,
                    succeeded='run_command', failed='recover')
        make_action('pick_object', PickObjectAction,
                    succeeded='run_command', failed='recover')
        make_action('place_object', PlaceObjectAction,
                    succeeded='run_command', failed='recover')
        make_action('deliver', DeliverAction,
                    succeeded='run_command', failed='recover')

def main():
    from smach_ros import IntrospectionServer

    rospy.init_node("state_machine")
    sm = test_smach()
    sm.userdata.hoge = 10
    insp = IntrospectionServer('server_name', sm, '/SM_ROOT')
    insp.start()
    result = sm.execute()
    print "result:", result
    print "userdata:", {k:sm.userdata[k] for k in sm.userdata.keys()}
    insp.stop()

if __name__ == '__main__':
    main()
