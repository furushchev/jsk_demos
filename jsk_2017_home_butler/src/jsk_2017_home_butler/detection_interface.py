#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import re
from collections import defaultdict
import math
import rospy
import message_filters as MF
from jsk_recognition_msgs.msg import ClassificationResult
from geometry_msgs.msg import PoseArray, PoseStamped
import tf2_ros
import tf2_geometry_msgs
assert tf2_geometry_msgs  # for linter


def norm(p):
    if isinstance(p, PoseStamped):
        p = p.pose
    d  = p.position.x * p.position.x
    d += p.position.y * p.position.y
    d += p.position.z * p.position.z
    return math.sqrt(d)


class UnknownObjectDatabase(object):
    _Properties = {}
    def __init__(self):
        pass

    def add_object(self, name, props):
        props = '|'.join(props.lower().strip().split())
        UnknownObjectDatabase._Properties.update({name: props})
        return True

    def get_props(self, name):
        if name in UnknownObjectDatabase._Properties:
            return UnknownObjectDatabase._Properties[name]
        else:
            return None

    def get_names(self):
        return UnknownObjectDatabase._Properties.keys()


class DetectionInterface(object):
    def __init__(self, queue_size=10, slop=0.1):
        self.tfl = tf2_ros.BufferClient("/tf2_buffer_server")
        ok = self.tfl.wait_for_server(timeout=rospy.Duration(10))
        if not ok:
            rospy.logfatal("Error: tf buffer server not found")
            return
        # self.subscribers = [
        #     MF.Subscriber("/ssd/ssd_object_detector/output/class",
        #                   ClassificationResult, queue_size=1),
        #     MF.Subscriber("/ssd/bounding_box_pose/output",
        #                   PoseArray, queue_size=1),
        # ]
        self.subscribers = [
            MF.Subscriber("/aggregator/aggregator/output/class",
                          ClassificationResult, queue_size=1),
            MF.Subscriber("/aggregator/aggregator/output/pose",
                          PoseArray, queue_size=1),
        ]
        sync = MF.ApproximateTimeSynchronizer(self.subscribers,
                                              queue_size=queue_size,
                                              slop=slop)
        sync.registerCallback(self.callback)

    def __del__(self):
        for s in self.subscribers:
            s.unregister()

    def sanitize(self, label):
        label = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', label)
        label = re.sub('([a-z0-9])([A-Z])', r'\1_\2', label).lower()
        label = label.replace('_', ' ')
        return label

    def callback(self, cls, pose):
        objects = defaultdict(list)
        for l, p in zip(cls.label_names, pose.poses):
            l = self.sanitize(l)
            # remove outliers
            if norm(p) > 0:
                ps = PoseStamped(header=pose.header, pose=p)
                objects[l].append(ps)

        self.objects = objects

        # find nearest poses
        nearest_objects = {}
        for l, poses in objects.items():
            nearest_pose = None
            min_distance = 1000000.0
            for ps in poses:
                try:
                    t = self.tfl.transform(ps, "hand_palm_link", rospy.Duration(10))
                except Exception as e:
                    rospy.logerr(str(e))
                    continue
                d = norm(t)
                if d < min_distance:
                    min_distance = d
                    nearest_pose = ps
            nearest_objects[l] = nearest_pose
        self.nearest_objects = nearest_objects

    def get_objects(self, labels=[], timeout=10, all_results=True):
        labels = [self.sanitize(l) for l in labels]
        timeout = int(timeout)
        self.objects = dict()
        for i in range(timeout):
            if rospy.is_shutdown():
                break
            if self.objects:
                if not labels:
                    return self.objects
                else:
                    ret_keys = []
                    for label in labels:
                        label = '|'.join(sorted(label.split('|'))).lower()
                        label_set = set(label.lower().strip().split('|'))
                        for obj_lab in self.objects.keys():
                            obj_lab_set = set(obj_lab.lower().strip().split('|'))
                            if label_set.issubset(obj_lab_set):
                                if all_results:
                                    return self.objects
                                else:
                                    ret_keys.append(obj_lab)
                    if ret_keys:
                        ret_keys = list(set(ret_keys))
                        return {k: self.objects[k] for k in ret_keys}
            rospy.sleep(1)

        # timeout
        rospy.logerr("Timed out")
        return dict()

    def get_nearest_objects(self, labels=[], timeout=10, all_results=True):
        labels = [self.sanitize(l) for l in labels]
        timeout = int(timeout)
        self.nearest_objects = dict()
        for i in range(timeout):
            if rospy.is_shutdown():
                break
            if self.nearest_objects:
                if not labels:
                    return self.nearest_objects
                else:
                    ret_keys = []
                    for label in labels:
                        label = '|'.join(sorted(label.split('|'))).lower()
                        label_set = set(label.lower().strip().split('|'))
                        for obj_lab in self.nearest_objects.keys():
                            obj_lab_set = set(obj_lab.lower().strip().split('|'))
                            if label_set.issubset(obj_lab_set):
                                if all_results:
                                    return self.nearest_objects
                                else:
                                    ret_keys.append(obj_lab)
                    if ret_keys:
                        ret_keys = list(set(ret_keys))
                        return {k: self.nearest_objects[k] for k in ret_keys}
            rospy.sleep(1)

        # timeout
        rospy.logerr("Timed out")
        return dict()


if __name__ == '__main__':
    rospy.init_node("detection_interface")
    d = DetectionInterface()
    print "d.get_objects()"
    print d.get_objects()
    labels = ["blue|tall"]
    print "d.get_objects(labels='%s', all_results=False)" % labels
    print d.get_nearest_objects(labels, all_results=False)
