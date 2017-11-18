#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
assert tf2_geometry_msgs
import message_filters as MF
from jsk_topic_tools import ConnectionBasedTransport
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox
from geometry_msgs.msg import PoseArray, PoseStamped


def distance(p1, p2):
    d =  (p1.position.x - p2.position.x) ** 2
    d += (p1.position.y - p2.position.y) ** 2
    d += (p1.position.z - p2.position.z) ** 2
    return math.sqrt(d)


class AnnotationAggregator(ConnectionBasedTransport):
    def __init__(self):
        super(AnnotationAggregator, self).__init__()

        self.approximate_sync = rospy.get_param("~approximate_sync", True)
        self.queue_size = rospy.get_param("~queue_size", 100)
        self.pose_tolerance = rospy.get_param("~pose_tolerance", 0.03)
        self.slop = rospy.get_param("~slop", 0.1)
        self.cache_duration = rospy.get_param("~cache_duration", 10.0)
        self.fixed_frame_id = rospy.get_param("~fixed_frame_id",
                                              "head_rgbd_sensor_rgb_frame")

        self.tfl = tf2_ros.BufferClient("/tf2_buffer_server")
        if not self.tfl.wait_for_server(rospy.Duration(10)):
            rospy.logfatal("timed out to wait /tf2_buffer_server")
            exit(1)

        self.cache = []

        self.pub_class = self.advertise(
            "~output/class", ClassificationResult, queue_size=1)
        self.pub_pose = self.advertise(
            "~output/pose", PoseArray, queue_size=1)
        self.pub_boxes = self.advertise(
            "~output/boxes", BoundingBoxArray, queue_size=1)

    def subscribe(self):
        self.sub_pose = [
            MF.Subscriber("~input/pose/class", ClassificationResult, queue_size=1),
            MF.Subscriber("~input/pose/pose", PoseArray, queue_size=1),
        ]
        self.sub_box = [
            MF.Subscriber("~input/box/class", ClassificationResult, queue_size=1),
            MF.Subscriber("~input/box/box", BoundingBoxArray, queue_size=1),
        ]
        if self.approximate_sync:
            sync_pose = MF.ApproximateTimeSynchronizer(
                self.sub_pose, queue_size=self.queue_size, slop=self.slop)
            sync_box = MF.ApproximateTimeSynchronizer(
                self.sub_box, queue_size=self.queue_size, slop=self.slop)
        else:
            sync_pose = MF.TimeSynchronizer(
                self.sub_pose, queue_size=self.queue_size)
            sync_box = MF.TimeSynchronizer(
                self.sub_box, queue_size=self.queue_size)
        sync_pose.registerCallback(self.callback_pose)
        sync_box.registerCallback(self.callback_box)

    def unsubscribe(self):
        for s in self.sub_pose + self.sub_box:
            s.unregister()

    def update_cache(self, stamp, new_pose, new_cls, new_clf):
        # match nearest pose
        self.cache.sort(key=lambda x: x[0])
        dups = []
        for i, e in enumerate(self.cache):
            pose = e[1]
            d = distance(pose, new_pose)
            if d < self.pose_tolerance:
                dups.append(i)

        # aggregate
        new_clss = {}
        for i in dups:
            t, p, clss = self.cache[i]
            new_clss.update(clss)
        new_clss.update({new_clf:new_cls})

        self.cache = [e for i, e in enumerate(self.cache) if i not in dups]
        self.cache.append((stamp, new_pose, new_clss))
        return

    def publish(self):
        if len(self.cache) == 0:
            return

        # cleanup old cache
        self.cache.sort(key=lambda x: x[0])
        now = rospy.Time.now()
        for i, e in enumerate(self.cache):
            t = e[0]
            if now - t < rospy.Duration(self.cache_duration):
                self.cache = self.cache[i:]
                rospy.loginfo("Cleared %d cache" % i)
                break
        stamp = self.cache[-1][0]
        rospy.loginfo("cached stamp: %.3f - %.3f" % ((now-self.cache[0][0]).to_sec(), (now-stamp).to_sec()))
        result = ClassificationResult()
        clf = sorted(set(reduce(lambda x,y: x+y, [e[2].keys() for e in self.cache])))
        result.classifier = '|'.join(clf)
        result.header.stamp = stamp
        result.header.frame_id = self.fixed_frame_id
        clss = map(lambda x: '|'.join(x), [sorted(e[2].values()) for e in self.cache])
        result.target_names = list(set(clss))
        poses = PoseArray(header=result.header)
        boxes = BoundingBoxArray(header=result.header)

        for i, c in enumerate(self.cache):
            p = c[1]
            cls = clss[i]
            idx = result.target_names.index(cls)
            result.labels.append(idx)
            result.label_names.append(cls)
            poses.poses.append(p)
            box = BoundingBox(header=result.header)
            box.pose = p
            box.dimensions.x = 0.1
            box.dimensions.y = 0.1
            box.dimensions.z = 0.1
            boxes.boxes.append(box)

        self.pub_class.publish(result)
        self.pub_pose.publish(poses)
        self.pub_boxes.publish(boxes)

    def callback_pose(self, cls, poses):
        rospy.loginfo("callback pose: %s %s" % (cls.header.stamp, cls.classifier))

        if len(cls.label_names) != len(poses.poses):
            rospy.logwarn("len(cls.label_names) != len(poses.poses)")
            return

        if poses.header.frame_id != self.fixed_frame_id:
            for i, p in enumerate(poses.poses):
                try:
                    ps = PoseStamped(header=poses.header, pose=p)
                    ps = self.tfl.transform(ps, self.fixed_frame_id)
                    poses.poses[i] = ps.pose
                except Exception as e:
                    rospy.logerr(e)
                    return

        for p, c in zip(poses.poses, cls.label_names):
            self.update_cache(cls.header.stamp, p, c, cls.classifier)
        self.publish()

    def callback_box(self, cls, boxes):
        rospy.loginfo("callback boxes: %s %s" % (cls.header.stamp, cls.classifier))

        if len(cls.label_names) != len(boxes.boxes):
            rospy.logwarn("len(cls.label_names) != len(boxes.boxes)")
            return

        if boxes.header.frame_id != self.fixed_frame_id:
            for i, b in enumerate(boxes.boxes):
                try:
                    ps = PoseStamped(header=b.header, pose=b.pose)
                    ps = self.tfl.transform(ps, self.fixed_frame_id)
                    boxes.boxes[i].pose = ps.pose
                except Exception as e:
                    rospy.logerr(e)
                    return

        for b, c in zip(boxes.boxes, cls.label_names):
            self.update_cache(cls.header.stamp, b.pose, c, cls.classifier)
        self.publish()


if __name__ == '__main__':
    rospy.init_node("annotation_aggregator")
    agg = AnnotationAggregator()
    rospy.spin()
