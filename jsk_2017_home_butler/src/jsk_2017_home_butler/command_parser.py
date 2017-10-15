#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from collections import OrderedDict
import luis
import rospy
import warnings
from jsk_2017_home_butler.action_loader import ActionLoader

class CommandParser(object):
    def __init__(self, app_id=None, app_key=None, endpoint=None):
        app_id = app_id or rospy.get_param("~app_id")
        app_key = app_key or rospy.get_param("~app_key")
        endpoint = endpoint or "westus.api.cognitive.microsoft.com"

        url = "https://{endpoint}/luis/v2.0/apps/{app_id}?subscription-key={app_key}&verbose=true&q=".format(
            endpoint=endpoint, app_id=app_id, app_key=app_key)
        self.luis = luis.Luis(url=url)

    def parse_command(self, text, ac_loader):
        assert isinstance(ac_loader, ActionLoader)

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

        rospy.loginfo("Received %d tasks and %d entities" % (len(tasks), len(entities)))

        used_entities = []
        commands = []
        for task in tasks:
            act = ac_loader.gen_action(task.type.split()[0])
            for entity in entities:
                if task.entity.strip().startswith(entity.entity):
                    act.name = entity.entity
                elif ' ' + entity.entity in task.entity.strip():
                    act.add_argument(entity.type, entity.entity)
                    used_entities.append(entity)
            commands.append(act)

        ambiguous_entities = list(set(entities) - set(used_entities))
        rospy.logwarn("Found %d ambiguous entities" % len(ambiguous_entities))

        ## TODO: Try to assign ambiguous task

        return commands


if __name__ == '__main__':
    ac_loader = ActionLoader()

    APPID = '345e17f0-0f2e-453b-a8af-1a86975da87c'
    APPKEY = 'bfececf25171466a97a61d600245f4ef'
    parser = CommandParser(APPID, APPKEY)
    query = "Could you please answer a question, look for Ethan in the corridor, and follow him."
    print "query:", query
    result = parser.parse_command(query, ac_loader)
    for i, r in enumerate(result):
        print "#%d ============" % i
        print r
