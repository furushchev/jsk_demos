#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from collections import OrderedDict
import luis
import rospy
import warnings

from jsk_2017_home_butler.msg import Command
from jsk_2017_home_butler.srv import ParseCommand, ParseCommandResponse


class CommandParser(object):
    def __init__(self, app_id, app_key):
        endpoint = rospy.get_param("~endpoint", "westus.api.cognitive.microsoft.com")
        app_id = app_id or rospy.get_param("~app_id")
        app_key = app_key or rospy.get_param("~app_key")

        uri = "https://{endpoint}/luis/v2.0/apps/{app_id}?subscription-key={app_key}&verbose=true&q=".format(
            endpoint=endpoint, app_id=app_id, app_key=app_key)
        self.luis = luis.Luis(url=uri)

        self.srv_com = rospy.Service(
            "parse_command", ParseCommand, self.parse_command_srv_cb)

    def parse_command_srv_cb(self, req):
        res = ParseCommandResponse()
        res.parsed_commands = self.parse_command(req.text)
        return res

    def parse_command(self, text):

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            result = self.luis.analyze(text)

        tasks = []
        entities = []
        for entity in result.entities:
            if entity.type.endswith("task"):
                tasks.append(entity)
            else:
                entities.append(entity)

        tasks.sort(key=lambda x: x.start_index)

        used_entities = []
        commands = []
        for task in tasks:
            cmd = Command(task_type=task.type.split()[0])
            for entity in entities:
                if task.entity.strip().startswith(entity.entity):
                    cmd.task_name = entity.entity
                elif ' ' + entity.entity in task.entity.strip():
                    cmd.arg_types.append(entity.type)
                    cmd.arg_values.append(entity.entity)
                    used_entities.append(entity)
            commands.append(cmd)

        ambiguous_entities = list(set(entities) - set(used_entities))
        rospy.logwarn("Found %d ambiguous entities" % len(ambiguous_entities))

        ## TODO: Try to assign ambiguous task

        return commands

if __name__ == '__main__':
    rospy.init_node("command_parser")
    parser = CommandParser()
    rospy.spin()

    # for standalone demo
    # APPID = '345e17f0-0f2e-453b-a8af-1a86975da87c'
    # APPKEY = 'bfececf25171466a97a61d600245f4ef'
    # parser = CommandParser(APPID, APPKEY)
    # query = "Could you please answer a question, look for Ethan in the corridor, and follow him."
    # print "query:", query
    # result = parser.parse_command(query)
    # for i, r in enumerate(result):
    #     print "#%d ============" % i
    #     print r
