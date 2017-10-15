#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import os
import lxml.etree as ET
import yaml
import rospkg

PKG_PATH = rospkg.RosPack().get_path("jsk_2017_home_butler")

_RESOURCE_CACHE = {}
_COMMAND_CACHE = {}


class ResourceLoader(object):
    def __init__(self, root_path=None):
        if root_path is None:
            root_path = os.path.join(PKG_PATH, "data")
        if not os.path.isdir(root_path):
            raise IOError("%s is not directory" % root_path)
        self.root_path = root_path

    def _load_names(self):
        path = os.path.join(self.root_path, "Names.xml")
        if not os.path.exists(path):
            raise IOError("%s not found" % path)

        tree = ET.parse(path)
        _RESOURCE_CACHE["names"] = [{
            'name': e.text.lower(),
            'gender': e.get("gender").lower() if e.get("gender") else "female",
        } for e in tree.xpath("//name")]
        return _RESOURCE_CACHE["names"]

    @property
    def names(self):
        if "names" not in _RESOURCE_CACHE:
            self._load_names()
        return _RESOURCE_CACHE["names"]

    def _load_locations(self):
        path = os.path.join(self.root_path, "Locations.xml")
        if not os.path.exists(path):
            raise IOError("%s not found" % path)

        tree = ET.parse(path)
        _RESOURCE_CACHE["locations"] = [{
            'room': e.getparent().get("name").lower(),
            'location': e.get("name").lower(),
        } for e in tree.xpath("//location")]
        return _RESOURCE_CACHE["locations"]

    @property
    def locations(self):
        if "locations" not in _RESOURCE_CACHE:
            self._load_locations()
        return _RESOURCE_CACHE["locations"]

    def _load_objects(self, difficulty="easy"):
        assert difficulty in ["easy", "moderate", "hard", "all"]
        if difficulty == "all":
            difficulty = ["easy", "moderate", "hard"]
        else:
            difficulty = [difficulty]
        path = os.path.join(self.root_path, "Objects.xml")
        if not os.path.exists(path):
            raise IOError("%s not found" % path)

        tree = ET.parse(path)
        _RESOURCE_CACHE["objects"] = [{
            'name': e.get("name").lower(),
            'category': e.getparent().get("name").lower(),
            'defaultLocation': e.getparent().get("defaultLocation").lower(),
            'room': e.getparent().get("room").lower(),
            'weight': int(e.get("weight")),
            'size': int(e.get("size")),
        } for e in tree.xpath("//object") if e.get("difficulty") in difficulty]
        return _RESOURCE_CACHE["objects"]

    @property
    def objects(self):
        if "objects" not in _RESOURCE_CACHE:
            self._load_objects()
        return _RESOURCE_CACHE["objects"]

    def _load_questions(self):
        path = os.path.join(self.root_path, "Questions.xml")
        if not os.path.exists(path):
            raise IOError("%s not found" % path)

        tree = ET.parse(path)
        _RESOURCE_CACHE["questions"] = [{
            "q": e.xpath("./q")[0].text,
            "a": e.xpath("./a")[0].text,
        } for e in tree.xpath("//question")]
        return _RESOURCE_CACHE["questions"]

    @property
    def questions(self):
        if "questions" not in _RESOURCE_CACHE:
            self._load_questions()
        return _RESOURCE_CACHE["questions"]


class CommandLoader(object):
    def __init__(self, yaml_path=None):
        if yaml_path is None:
            yaml_path = os.path.join(PKG_PATH, "data", "command_easy.yaml")
        if not os.path.exists(yaml_path):
            raise IOError("%s not found" % yaml_path)
        self.yaml_path = yaml_path

    def load_basic_verbs(self):
        with open(self.yaml_path, "r") as f:
            _COMMAND_CACHE["basic_verbs"] = yaml.load(f)["verbs"]
        return _COMMAND_CACHE["basic_verbs"]

    @property
    def basic_verbs(self):
        if "basic_verbs" not in _COMMAND_CACHE:
            self.load_basic_verbs()
        return _COMMAND_CACHE["basic_verbs"]

    def load_basic_commands(self):
        with open(self.yaml_path, "r") as f:
            _COMMAND_CACHE["basic_commands"] = yaml.load(f)["commands"]
        return _COMMAND_CACHE["basic_commands"]

    @property
    def basic_commands(self):
        if "basic_commands" not in _COMMAND_CACHE:
            self.load_basic_commands()
        return _COMMAND_CACHE["basic_commands"]


if __name__ == '__main__':
    import pprint
    pp = pprint.PrettyPrinter(indent=2)

    loader = ResourceLoader()

    print "=================================================="
    print "parsed from Names.xml"
    pp.pprint(loader.names)

    print "=================================================="
    print "parsed from Locations.xml"
    pp.pprint(loader.locations)

    print "=================================================="
    print "parsed from Objects.xml"
    pp.pprint(loader.objects)

    print "=================================================="
    print "parsed from Questions.xml"
    pp.pprint(loader.questions)

    loader = CommandLoader()
    print "=================================================="
    print "parsed basic verbs from command_easy.yaml"
    pp.pprint(loader.basic_verbs)

    print "=================================================="
    print "parsed basic commands from command_easy.yaml"
    pp.pprint(loader.basic_commands)
