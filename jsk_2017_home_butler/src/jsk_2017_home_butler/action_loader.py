#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import os
import rospkg
import yaml
from jsk_2017_home_butler.action import Action

PKG_PATH = rospkg.RosPack().get_path("jsk_2017_home_butler")


class ActionLoader(object):
    def __init__(self, root_path=None):
        if root_path is None:
            root_path = os.path.join(PKG_PATH, "data")
        if not os.path.isdir(root_path):
            raise IOError("directory '%s' not found" % root_path)
        def_path = os.path.join(root_path, "action_definition.yaml")
        self.load_definition(def_path)

    def load_definition(self, def_path):
        with open(def_path, "r") as f:
            data = yaml.load(f.read())
        defs = {}
        for d in data["action_definitions"]:
            defs[d["type"]] = d
        self.action_definitions = defs

    def gen_action(self, ac_type):
        d = self.action_definitions[ac_type]
        return Action(ac_type, d["name"], d["arguments"], d["format"])


if __name__ == '__main__':
    loader = ActionLoader()
    act = loader.gen_action("pick")
    print act
    act.name = "pickup"
    act.add_argument("object", "mochi")
    print act.missing_arguments
    act.add_argument("location", "drawer")
    print act
