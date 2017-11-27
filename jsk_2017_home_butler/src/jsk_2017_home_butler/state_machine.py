#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from actionlib_msgs.msg import GoalStatus
import re
import rospy
from smach import StateMachine
from smach_ros import SimpleActionState

from jsk_2017_home_butler.utils import camel_to_snake
from jsk_2017_home_butler.actions import PRIMITIVE_ACTIONS
from jsk_2017_home_butler.actions import ListenCommandAction
from jsk_2017_home_butler.actions import WaitForCommandAction
from jsk_2017_home_butler.actions import RunCommandAction
from jsk_2017_home_butler.actions import RecoverAction

from jsk_2017_home_butler.msg import AnswerQuestionAction
from jsk_2017_home_butler.msg import FindObjectAction
from jsk_2017_home_butler.msg import FindPersonAction
from jsk_2017_home_butler.msg import DeliverAction
from jsk_2017_home_butler.msg import GoAction
from jsk_2017_home_butler.msg import PickAction
from jsk_2017_home_butler.msg import PlaceAction
from jsk_2017_home_butler.msg import SpeakAction


def hoge():
    def goal_cb(ud, goal):
        rospy.logwarn("GOAL: %s" % type(goal))
        return goal

    def result_cb(userdata, status, result):
        rospy.logwarn("UD: %s, ST: %s, res: %s" % (userdata, status, result))
        if status == GoalStatus.SUCCEEDED:
            rospy.logwarn("SUCCEEDED!")
            slots = result.__slots__
            for ex in ["header", "status"]:
                if ex in slots:
                    slots.remove(ex)
            userdata.result = {k: getattr(result, k) for k in slots}
        rospy.logwarn("RESULT: %s" % userdata.result)

    from actionlib_tutorials.msg import FibonacciAction
    StateMachine.add(
        'fib',
        SimpleActionState('fibonacci', FibonacciAction,
                          goal_cb=goal_cb,
                          result_cb=result_cb,
                          goal_slots=['order'],
                          result_slots=['sequence'],
                          input_keys=['result'],
                          output_keys=['result']),
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

def make_action(name, spec,
                succeeded='succeeded', failed='failed'):

    def goal_cb(userdata, goal):
        assert isinstance(userdata.arguments, dict)
        argkeys = set(userdata.arguments.keys())
        goalkeys = set(goal.__slots__)
        diffkeys = goalkeys.difference(argkeys)
        if diffkeys:
            rospy.logwarn("Arguments are missing?: %s" % diffkeys)
        for k, v in userdata.arguments.items():
            if hasattr(goal, k):
                setattr(goal, k, v)
        return goal

    def result_cb(userdata, status, result):
        if status == GoalStatus.SUCCEEDED:
            slots = result.__slots__
            for ex in ["header", "status"]:
                if ex in slots:
                    slots.remove(ex)
            userdata.result = {k: getattr(result, k) for k in slots}

    StateMachine.add(
        name,
        SimpleActionState(camel_to_snake(name), spec,
                          goal_cb=goal_cb,
                          result_cb=result_cb,
                          input_keys=['arguments', 'result'],
                          output_keys=['result'],
                          server_wait_timeout=rospy.Duration(10),
                          exec_timeout=rospy.Duration(90),
                          preempt_timeout=rospy.Duration(60)),
        transitions={'succeeded': succeeded,
                     'aborted': failed,
                     'preempted': failed})

def make_state_machine():
    sm = StateMachine(['succeeded', 'failed'])
    with sm:
        StateMachine.add('WaitForCommand', WaitForCommandAction(),
                         transitions={'succeeded': 'ListenCommand',
                                      'failed': 'WaitForCommand'})

        StateMachine.add('ListenCommand',
                         ListenCommandAction(),
                         transitions={'succeeded': 'RunCommand',
                                      'failed': 'WaitForCommand'})
                         # remapping={'query': 'query',
                         #            'actions': 'actions'})

        transitions = {'succeeded': 'succeeded'}
        for ac in PRIMITIVE_ACTIONS:
            transitions.update({ac: ac})
        StateMachine.add('RunCommand',
                         RunCommandAction(),
                         transitions=transitions)

        StateMachine.add('Recover',
                         RecoverAction(),
                         transitions={'succeeded': 'succeeded'})

        # action primitives
        make_action('FindPerson', FindPersonAction,
                    succeeded='RunCommand', failed='Recover')
        make_action('Speak', SpeakAction,
                    succeeded='RunCommand', failed='Recover')
        make_action('Go', GoAction,
                    succeeded='RunCommand', failed='Recover')
        make_action('FindObject', FindObjectAction,
                    succeeded='RunCommand', failed='Recover')
        make_action('AnswerQuestion', AnswerQuestionAction,
                    succeeded='RunCommand', failed='Recover')
        make_action('Pick', PickAction,
                    succeeded='RunCommand', failed='Recover')
        make_action('Place', PlaceAction,
                    succeeded='RunCommand', failed='Recover')
        make_action('Deliver', DeliverAction,
                    succeeded='RunCommand', failed='Recover')

    return sm


def main():
    from smach_ros import IntrospectionServer

    rospy.init_node("state_machine")

    rospy.set_param("~app_id", '48a5ef97-c9c5-4986-85c7-f6f5ef6f4bbc')
    rospy.set_param("~app_key", 'bfececf25171466a97a61d600245f4ef')

    # sm = test_smach()
    sm = make_state_machine()
    sm.userdata.hoge = 10
    insp = IntrospectionServer('server_name', sm, '/SM_ROOT')
    insp.start()
    result = sm.execute()
    print "result:", result
    print "userdata:", {k:sm.userdata[k] for k in sm.userdata.keys()}
    insp.stop()

if __name__ == '__main__':
    main()
