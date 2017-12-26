#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

"""
This program is aimed for providing:
* Category estimation based on bayesian network inference using aggregated annotations
* Sensor reliability aware detection model used bayesian estimation
"""

import numpy as np

from libpgm.discretebayesiannetwork import DiscreteBayesianNetwork

import rospy
from jsk_topic_tools import ConnectionBasedTransport
from jsk_robot_startup.lifelog import LoggerBase
from jsk_recognition_msgs.msg import ClassificationResult


class BayesCategoryEstimator(LoggerBase):
    def __init__(self):
        super(BayesCategoryEstimator, self).__init__()
        ## TODO: construct DAG

    def learn(self):
        pass

    def infer(self, result):
        category = result[0]  # FIXME
        return category


class EstimatorNode(LoggerBase, ConnectionBasedTransport):
    def __init__(self):
        super(EstimatorNode, self).__init__()

        self.rel = SensorReliabilityEstimator()
        self.cat = BayesCategoryEstimator()

        self.pub = self.advertise("~output",
                                  ClassificationResult,
                                  queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber("~input", ClassificationResult,
                                    self.raw_class_cb)

    def unsubscribe(self):
        if self.sub:
            self.sub.shutdown()
        self.sub = None

    def raw_class_cb(self, msg):
        pass


if __name__ == '__main__':
    rospy.init_node("bayes_estimator")
    n = EstimatorNode()
    rospy.spin()
