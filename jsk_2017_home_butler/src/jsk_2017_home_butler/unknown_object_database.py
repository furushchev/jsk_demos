#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import os
import lxml.etree as ET
import rospkg
from jsk_2017_home_butler.resource_loader import ResourceLoader

PKG_PATH = rospkg.RosPack().get_path('jsk_2017_home_butler')
_DB_CACHE = None


class UnknownObjectDatabase(object):
    def __init__(self, root_path=None):
        if root_path is None:
            root_path = os.path.join(PKG_PATH, 'data')
        if not os.path.isdir(root_path):
            raise IOError("%s is not a directory" % root_path)
        self.db_path = os.path.join(root_path, 'UnknownObjects.xml')

        self.resource_loader = ResourceLoader(root_path)

    @property
    def tree(self):
        global _DB_CACHE
        if _DB_CACHE is None:
            parser = ET.XMLParser(remove_blank_text=True)
            with open(self.db_path, "r") as f:
                _DB_CACHE = ET.fromstring(f.read(), parser=parser)
        return _DB_CACHE

    def update(self, name, properties, location):
        tree = self.tree
        obj = tree.xpath("//object[@name='%s' and @defaultLocation='%s']" % (name, location))
        if obj:
            if len(obj) > 1:
                rospy.logwarn("multiple definition for object '%s' at '%s'" % (name, location))
            obj = obj[0]
            tree.remove(obj)
            orig_props = {e.get("type"): e.get("value") for e in obj.getchildren()}
        else:
            orig_props = {}
        obj = ET.Element(
            "object",
            name=name,
            defaultLocation=location
        )
        orig_props.update(properties)
        properties = orig_props
        for t, v in properties.items():
            obj.append(ET.Element(
                "property",
                type=t, value=v))
        tree.append(obj)

        doc = ET.tostring(
            tree,
            encoding='utf-8',
            xml_declaration=True,
            pretty_print=True,
            with_tail=True,
            with_comments=True)

        with open(self.db_path, "w") as f:
            f.write(doc)

    @property
    def objects(self):
        return [{
            "name": e.get("name"),
            "properties": {c.get("type"): c.get("value") for c in e.getchildren()},
            "location": e.get("defaultLocation")
        } for e in self.tree.xpath("//object")]

    def add_object(self, name, properties, location):
        # assert location
        found_loc = False
        for loc in self.resource_loader.locations:
            if location in loc["location"]:
                found_loc = True
                break
        if not found_loc:
            raise ValueError("Location %s is not found in database" % location)

        self.update(name, properties, location)

    def get_properties(self, name, location):
        obj = filter(lambda e: e["name"] == name and e["location"] == location, self.objects)
        if obj:
            return obj[0]["properties"]
        return None

    def get_names(self):
        return [e["name"] for e in self.objects]

if __name__ == '__main__':
    db = UnknownObjectDatabase()
    print db.get_names()
    print db.get_properties("mochi", "kitchen shelf")
    db.add_object("apple",
                  {"color": "red",
                   "volume": "small",
                   "primitive": "circle"},
                  "kitchen table")
    print db.get_names()
    print db.get_properties("apple", "kitchen table")

