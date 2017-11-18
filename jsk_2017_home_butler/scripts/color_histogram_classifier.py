#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import rospy
import numpy as np
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_msgs.msg import ColorHistogramArray
from jsk_recognition_msgs.msg import ClassificationResult


class ColorHistogramClassifier(ConnectionBasedTransport):
    def __init__(self):
        super(ColorHistogramClassifier, self).__init__()
        self.pub_class = self.advertise(
            "~output", ClassificationResult, queue_size=1)

    def subscribe(self):
        self.sub_hist = rospy.Subscriber(
            "~input", ColorHistogramArray, self.callback, queue_size=1)

    def unsubscribe(self):
        self.sub_hist.unregister()

    def callback(self, msg):
        if len(msg.histograms) == 0:
            return
        elif len(msg.histograms[0].histogram) != 10 + 2:
            rospy.logerr("histogram bin size must be 10 + 2")
            return
        cls = ClassificationResult(header=msg.header)
        cls.classifier = "color_histogram_classifier"
        cls.target_names = [
            "red", "yellow", "green", "blue",
            "white", "black",
        ]

        for h in msg.histograms:
            idx = np.array(h.histogram).argmax()
            if idx in [0,1,2,9]:
                idx = 0
            elif idx in [3]:
                idx = 1
            elif idx in [4,5]:
                idx = 2
            elif idx in [6,7,8]:
                idx = 3
            elif idx == 10:
                idx = 4
            elif idx == 11:
                idx = 5
            cls.labels.append(idx)
            cls.label_names.append(cls.target_names[idx])

        self.pub_class.publish(cls)


if __name__ == '__main__':
    rospy.init_node("color_histogram_classifier")
    clf = ColorHistogramClassifier()
    rospy.spin()
