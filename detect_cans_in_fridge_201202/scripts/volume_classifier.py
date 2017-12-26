#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import rospy
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import ClassificationResult
from gpsr.cfg import BoundingBoxVolumeClassifierConfig
from dynamic_reconfigure.server import Server




class VolumeClassifier(ConnectionBasedTransport):
    def __init__(self):
        super(VolumeClassifier, self).__init__()
        self.pub_class = self.advertise(
            "~output", ClassificationResult, queue_size=1)
        self.srv = Server(
            VolumeClassifierConfig, self.config_callback)

    def subscribe(self):
        self.sub_boxes = rospy.Subscriber(
            "~input", BoundingBoxArray, self.bbox_cb)

    def unsubscribe(self):
        self.sub_boxes.unregister()

    def config_callback(self, config, level):
        self.tall_threshold = config.tall_threshold
        self.big_threshold = config.big_threshold
        return config

    def bbox_cb(self, bboxes):
        msg = ClassificationResult()
        msg.header = bboxes.header
        msg.target_names = ["short", "tall", "small", "big"]
        msg.classifier = "volume_classifier"
        for i, b in enumerate(bboxes.boxes):
            long_edge = max(b.dimensions.x, max(b.dimensions.y, b.dimensions.z))
            avg_edge = (b.dimensions.x + b.dimensions.y + b.dimensions.z) / 3.0
            volume = b.dimensions.x * b.dimensions.y * b.dimensions.z
            rospy.loginfo("%d: volume: %f, long: %f" % (i, volume, long_edge))
            val = avg_edge / long_edge
            if val > self.tall_threshold:
                if volume > self.big_threshold:
                    msg.labels.append(3)
                    msg.label_names.append("big")
                    msg.label_proba.append(1.0)
                else:
                    msg.labels.append(2)
                    msg.label_names.append("small")
                    msg.label_proba.append(1.0)
            else:
                if volume > self.big_threshold:
                    msg.labels.append(1)
                    msg.label_names.append("tall")
                    msg.label_proba.append(1.0)
                else:
                    msg.labels.append(0)
                    msg.label_names.append("short")
                    msg.label_proba.append(1.0)
        self.pub_class.publish(msg)


if __name__ == '__main__':
    rospy.init_node("volume_classifier")
    c = VolumeClassifier()
    rospy.spin()
