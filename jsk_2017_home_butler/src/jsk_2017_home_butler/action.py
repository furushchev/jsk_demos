#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import random


class Action(object):
    def __init__(self, ac_type, ac_name, arguments, str_format=None):
        self.ac_type = ac_type
        self.ac_name = ac_name
        self.arguments = {t:'' for t in arguments}
        self.str_format = str_format

    def __str__(self):
        try:
            fmt = self.str_format.format(name=self.name, **self.arguments)
        except:
            fmt = "<Action %s" % self.ac_name
            for t, v in self.arguments.items():
                fmt += " %s=%s" % (t, v)
            fmt += ">"
        return fmt

    @property
    def missing_arguments(self):
        return [t for t, v in self.arguments.items() if not v]

    @property
    def type(self):
        return self.ac_type

    @property
    def name(self):
        if isinstance(self.ac_name, str):
            return self.ac_name
        elif isinstance(self.ac_name, unicode):
            return self.ac_name
        elif isinstance(self.ac_name, list):
            random.shuffle(self.ac_name)
            return self.ac_name[0]
        else:
            raise ValueError("invalid name: '%s' for type '%s'" % (self.ac_name, self.ac_type))

    @name.setter
    def name(self, val):
        assert (isinstance(val, str) or
                isinstance(val, unicode) or
                isinstance(val, list)), "name '%s'(%s) must be str or list" % (val, type(val))
        self.ac_name = val

    def add_argument(self, arg_type, arg_value):
        if arg_type not in self.arguments.keys():
            raise KeyError("Action %s doesn't have argument type '%s'" % (self.ac_name, arg_type))
        if not arg_value:
            raise ValueError("Value cannot be empty")
        self.arguments[arg_type] = arg_value
