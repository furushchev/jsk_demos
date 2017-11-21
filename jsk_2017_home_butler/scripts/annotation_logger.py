#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from collections import OrderedDict
import os
import rospkg
import rospy
import string
from threading import Event, Lock, Thread
from jsk_recognition_msgs.msg import ClassificationResult

PKGDIR = rospkg.RosPack().get_path("jsk_2017_home_butler")


class WriteBuffer(Thread):
    def __init__(self, length, out, rate, formatter=string.join):
        self.lock = Lock()
        self.event = Event()
        self.buffer = OrderedDict()
        self.length = length
        self.formatter = formatter
        self.out = out
        self.rate = rate

        super(WriteBuffer, self).__init__()
        self.daemon = True

    def stop(self):
        self.event.set()

    def add(self, cls):
        with self.lock:
            key = cls.header.stamp
            try:
                self.buffer[key] += [cls]
            except:
                self.buffer[key] = [cls]

    def get(self):
        with self.lock:
            data = list()
            while len(self.buffer) > self.length:
                data.append(self.buffer.popitem(last=False))
        return data

    def run(self):
        rospy.loginfo("writing into %s" % self.out)
        with open(self.out, "a") as f:
            while not self.event.wait(self.rate):
                data = self.get()
                for line in data:
                    f.write(self.formatter(*line))
                    f.write(os.linesep)


class AnnotationLogger(object):
    def __init__(self):
        super(AnnotationLogger, self).__init__()

        topics = list()
        if rospy.get_param("~auto_topic_detection", True):
            topics = self.find_class_topics()
        else:
            class_num = rospy.get_param("~class_topic_num", 0)
            for i in range(class_num):
                topics.append("~input/class%d" % i)

        class_dir = os.path.join(PKGDIR, "data", "categories")
        if not os.path.isdir(class_dir):
            os.makedirs(class_dir)
            rospy.loginfo("Created directory: %s" % class_dir)
        buffer_path = os.path.join(class_dir, rospy.get_param("~category") + ".class")
        self.buffer = WriteBuffer(length=10, out=buffer_path, rate=1.0, formatter=self.formatter)
        rospy.on_shutdown(self.on_shutdown)
        self.buffer.start()

        self.subscribers = list()
        for t in topics:
            self.subscribers.append(rospy.Subscriber(t, ClassificationResult, self.on_message))

    def find_class_topics(self):
        return [n for n, t in rospy.get_published_topics() if t == ClassificationResult._type]

    def formatter(self, stamp, data):
        line = [str(stamp)]
        for cls in data:
            if cls.label_names:
                line.append("%s=%s" % (cls.classifier, cls.label_names[0]))
        return ",".join(line)

    def on_shutdown(self):
        self.buffer.stop()

    def on_message(self, msg):
        self.buffer.add(msg)


if __name__ == '__main__':
    rospy.init_node("annotation_logger")
    logger = AnnotationLogger()
    rospy.spin()
