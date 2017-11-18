#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import copy
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


def strip_frame(frame):
    if frame.startswith("/"):
        return frame[1:]
    else:
        return frame


class AnnotationAggregator(ConnectionBasedTransport):
    def __init__(self):
        super(AnnotationAggregator, self).__init__()

        self.approximate_sync = rospy.get_param("~approximate_sync", True)
        self.queue_size = rospy.get_param("~queue_size", 100)
        self.pose_tolerance = rospy.get_param("~pose_tolerance", 0.03)
        self.slop = rospy.get_param("~slop", 0.1)
        self.cache_duration = rospy.get_param("~cache_duration", 10.0)
        self.fixed_frame_id = strip_frame(
            rospy.get_param("~fixed_frame_id", "head_rgbd_sensor_rgb_frame"))

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
        pose_num = rospy.get_param("~pose_annotators_num", 0)
        bbox_num = rospy.get_param("~bbox_annotators_num", 0)

        self.subscribers = list()
        self.synchronizers = list()

        # pose based annotators
        sub_pose = MF.Subscriber("~input/pose/pose", PoseArray, queue_size=1)
        self.subscribers.append(sub_pose)
        for i in range(pose_num):
            sub_cls = MF.Subscriber(
                "~input/pose/class{}".format(i), ClassificationResult, queue_size=1)
            if self.approximate_sync:
                sync = MF.ApproximateTimeSynchronizer(
                    [sub_pose, sub_cls], queue_size=self.queue_size, slop=self.slop)
            else:
                sync = MF.TimeSynchronizer(
                    [sub_pose, sub_cls], queue_size=self.queue_size)
            sync.registerCallback(self.callback_pose)
            self.subscribers.append(sub_cls)
            self.synchronizers.append(sync)

        # bbox based annotators
        sub_bbox = MF.Subscriber("~input/bbox/bbox", BoundingBoxArray, queue_size=1)
        self.subscribers.append(sub_bbox)
        for i in range(bbox_num):
            sub_cls = MF.Subscriber(
                "~input/bbox/class{}".format(i), ClassificationResult, queue_size=1)
            if self.approximate_sync:
                sync = MF.ApproximateTimeSynchronizer(
                    [sub_bbox, sub_cls], queue_size=self.queue_size, slop=self.slop)
            else:
                sync = MF.TimeSynchronizer(
                    [sub_bbox, sub_cls], queue_size=self.queue_size)
            sync.registerCallback(self.callback_bbox)
            self.subscribers.append(sub_cls)
            self.synchronizers.append(sync)

        rospy.loginfo("Subscribed %d pose annotators / %d bbox annotators" % (pose_num, bbox_num))

    def unsubscribe(self):
        for s in self.subscribers:
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

    def publish(self):
        # cleanup old cache
        self.cache.sort(key=lambda x: x[0], reverse=True)
        now = rospy.Time.now()
        for i, e in enumerate(self.cache):
            t = e[0]
            rospy.loginfo("diff: %f" % (now - t).to_sec())
            if (now - t).to_sec() > self.cache_duration:
                rospy.loginfo("Clear %d caches" % (len(self.cache)-i))
                self.cache = self.cache[:i]
                break
        self.cache.reverse()

        if len(self.cache) == 0:
            return

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

    def callback_pose(self, poses, cls):
        poses = copy.deepcopy(poses)
        rospy.loginfo("callback pose: %s %s" % (cls.header.stamp, cls.classifier))

        if len(cls.label_names) != len(poses.poses):
            rospy.logwarn("len(cls.label_names) != len(poses.poses)")
            return

        frame_id = strip_frame(poses.header.frame_id)
        if frame_id != self.fixed_frame_id:
            for i, p in enumerate(poses.poses):
                try:
                    ps = PoseStamped(pose=p)
                    ps.header.frame_id = frame_id
                    ps.header.stamp = poses.header.stamp
                    ps = self.tfl.transform(ps, self.fixed_frame_id)
                    poses.poses[i] = ps.pose
                except Exception as e:
                    rospy.logerr(e)
                    return

        for p, c in zip(poses.poses, cls.label_names):
            self.update_cache(cls.header.stamp, p, c, cls.classifier)
        self.publish()

    def callback_bbox(self, boxes, cls):
        boxes = copy.deepcopy(boxes)
        rospy.loginfo("callback boxes: %s %s" % (cls.header.stamp, cls.classifier))

        if len(cls.label_names) != len(boxes.boxes):
            rospy.logwarn("len(cls.label_names) != len(boxes.boxes)")
            return

        for i, b in enumerate(boxes.boxes):
            frame_id = strip_frame(b.header.frame_id)
            if frame_id != self.fixed_frame_id:
                try:
                    ps = PoseStamped(pose=b.pose)
                    ps.header.frame_id = frame_id
                    ps.header.stamp = b.header.stamp
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
