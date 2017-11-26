#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import traceback
import rospy
from smach import State
from jsk_2017_home_butler.action_loader import ActionLoader
from jsk_2017_home_butler.preprocessor import CommandPreprocessor
from jsk_2017_home_butler.command_parser import CommandParser
from jsk_2017_home_butler.interpolator import CommandInterpolator
from jsk_2017_home_butler.interpolator import CommandInterpolateError
from jsk_2017_home_butler.interpolator import UnknownSymbolError
from jsk_2017_home_butler.interpolator import SymbolPropertyMissingError
from jsk_2017_home_butler.unknown_object_database import UnknownObjectDatabase
from jsk_2017_home_butler.utils import SpeechMixin


class ListenCommandAction(State, SpeechMixin):
    def __init__(self, app_id=None, app_key=None):

        self.preprocessor = CommandPreprocessor()
        self.parser = CommandParser(app_id=app_id, app_key=app_key)
        self.interpolator = CommandInterpolator()

        self.action_loader = ActionLoader()

        State.__init__(self,
                       outcomes=['succeeded', 'failed'],
                       output_keys=['query', 'actions'])

    def listen_query(self):
        for i in range(3):
            if i == 0:
                query = "May I help you?"
            else:
                query = "Please say again?"
            answer = self.ask(query, duration=15.0, threshold=0.6)
            if answer:
                return answer
            self.say("Sorry, I cannot get you.")
        return ""

    def recognize(self, query):
        rospy.loginfo("Query: %s", query)

        query = self.preprocessor.process(query)
        if not query:
            rospy.logerr("Failed to preprocess")
            return None
        rospy.loginfo("Preprocessed: %s" % query)

        actions = self.parser.parse_command(query, self.action_loader)
        if not actions:
            rospy.logerr("Failed to parse actions")
            return None

        return actions

    def interpolate(self, actions):
        while True:
            if rospy.is_shutdown():
                break
            try:
                actions = self.interpolator.interpolate(actions, self.action_loader)
                break
            except UnknownSymbolError as e:
                rospy.logerr(str(e))
                rospy.logerr(traceback.format_exc())
                self.say("Sorry, %s." % str(e))
                valid = e.valid_commands
                cmd = e.command
                if "where" in str(e):
                    q = "Where should I do %s" % cmd.name
                    key = "location"
                    rospy.loginfo("q: %s" % q)
                    answer = self.ask(q, grammar="gpsr")
                    rospy.loginfo("a: %s" % answer)
                    # answer = "living drawer"  # FIXME: temp fix for testing
                    cmd.add_argument(key, answer)
                elif "what" in str(e):
                    q = "What is %s?" % cmd.arguments["object"]
                    rospy.loginfo("q: %s" % q)
                    answer = self.ask(q, grammar="gpsr")
                    rospy.loginfo("a: %s" % answer)
                    # TODO: ask location if not provided
                    # TODO: need parse props for the object
                    props = {"color": "red", "volume": "small"} # FIXME: temp fix for testing
                    db = UnknownObjectDatabase()
                    db.add_object(cmd.arguments["object"], props, cmd.arguments["location"])
                # update actions
                rospy.loginfo("valid: %s" % valid)
                rospy.loginfo("cmd: %s" % cmd)
                rospy.loginfo("commands: %s" % actions)
                actions = valid + [cmd] + e.all_commands[1:]
            except SymbolPropertyMissingError as e:
                rospy.logerr(str(e))
                rospy.logerr(traceback.format_exc())
                self.say("Sorry, %s" % str(e))
                cmd = e.command
                if "where" in str(e):
                    loc = self.ask("Where is %s located?" % cmd.arguments["object"], grammar="gpsr")
                    cmd.add_argument("location", loc)
                actions = e.valid_commands + [cmd] + e.all_commands[1:]
            except CommandInterpolateError as e:
                rospy.logerr(e)
                rospy.logerr(traceback.format_exc())
                self.say("Sorry, %s." % str(e))
                return list()

        return actions

    def execute(self, userdata=None, query=None):
        self.say("Hi!")
        rospy.sleep(0.3)

        if query is None:
            query = self.listen_query()
        if not query:
            return 'failed'
        self.say("You said: %s" % query)
        userdata.query = query

        self.say("Let me see...", wait=False)

        actions = self.recognize(query)
        if not actions:
            return 'failed'
        rospy.loginfo("Parsed: %s" % [str(ac) for ac in actions])

        actions = self.interpolate(actions)
        if not actions:
            return 'failed'
        rospy.loginfo("Interpolated: %s" % [str(ac) for ac in actions])
        userdata.actions = actions

        return 'succeeded'


if __name__ == '__main__':
    import sys
    from smach import UserData
    from smach import StateMachine

    rospy.init_node("listen_command")

    rospy.set_param("~app_id", '48a5ef97-c9c5-4986-85c7-f6f5ef6f4bbc')
    rospy.set_param("~app_key", 'bfececf25171466a97a61d600245f4ef')

    if len(sys.argv) > 1:
        query = ' '.join(sys.argv[1:])
        ac = ListenCommandAction()
        data = UserData()
        result = ac.execute(userdata=data, query=query)
        print "result:", result, "actions:", data.actions
        sys.exit(0)

    sm = StateMachine(outcomes=['succeeded', 'failed'])
    with sm:
        StateMachine.add("listen_command", ac,
                         transitions={'succeeded': 'succeeded',
                                      'failed': 'failed'},
                         remapping={'query': 'query',
                                    'actions': 'actions'})
    result = sm.execute()

    print "result:", result, "query:", sm.userdata.query
    print "actions:", sm.userdata.actions
