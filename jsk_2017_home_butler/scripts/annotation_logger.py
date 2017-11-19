#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from collections import OrderedDict
import os
import rospy
import string
from threading import Lock, Thread
from jsk_recognition_msgs.msg import ClassificationResult


class WriteBuffer(Thread):
    def __init__(self, length, out, formatter=string.join):
        self.lock = Lock()
        self.buffer = OrderedDict()
        self.length = length
        self.formatter = formatter
        self.out = out

        super(WriteBuffer, self).__init__()
        self.daemon = True

    def add(self, cls):
        with self.lock:
            if cls.header.stamp in self.buffer:
                self.buffer[cls.header.stamp] = [cls]
            else:
                self.buffer[cls.header.stamp] += [cls]

    def get(self):
        with self.lock:
            data = list()
            while len(self.buffer) > self.length:
                data.append(self.buffer.popitem(last=False))
        return data

    def run(self):
        with open(out, "a") as f:
            pass


class AnnotationLogger(object):
    def __init__(self):
        super(AnnotationLogger, self).__init__()

        self.topics = list()
        if rospy.get_param("~auto_topic_detection", True):
            self.topics = self.find_class_topics()
        else:
            class_num = rospy.get_param("~class_topic_num", 0)
            for i in range(class_num):
                self.topics.append("~input/class%d" % i)

        self.buffer_duration = rospy.get_param("~buffer_duration", 10)


    def find_class_topics(self):
        return [n for n, t in rospy.get_published_topics() if t == ClassificationResult._type]


if __name__ == '__main__':
    pass
