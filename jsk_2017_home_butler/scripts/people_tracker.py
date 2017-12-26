#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import angles
from collections import defaultdict
from jsk_topic_tools import ConnectionBasedTransport
import message_filters as MF
import numpy as np
import rospy
import tf.transformations as T
import tf2_ros
import tf2_geometry_msgs
assert tf2_geometry_msgs

from geometry_msgs.msg import PoseArray, PoseStamped
from jsk_recognition_msgs.msg import ClassificationResult
from opencv_apps.msg import FaceArrayStamped


class Kalman(object):
    def __init__(self, state_dim=2, meas_dim=2, P_var=1., Q_var=1e-5, R_var=1e-2):
        super(Kalman, self).__init__()
        self.B = np.eye(state_dim)
        self.F = np.eye(state_dim)
        self.H = np.eye(meas_dim, state_dim)
        self.P = np.eye(state_dim) * P_var
        self.Q = np.eye(state_dim) * Q_var
        self.R = np.eye(meas_dim)  * R_var
        self.S = np.eye(state_dim)

        self.u = np.zeros((state_dim,))
        self.z = np.zeros((meas_dim,))
        self.x = self.H.T.dot(self.z)

        self.state_dim = state_dim
        self.meas_dim = meas_dim

    def predict(self, dt=1):
        self.x = self.F.dot(self.x) + self.B.dot(self.u)
        self.P = self.F.dot(self.P.dot(self.F.T)) + self.Q

    def update(self, z):
        self.z = z  # for evaluate error

        # y = z - Hx
        y = z - self.H.dot(self.x)

        # S = H P H^ + R
        self.S = self.H.dot(self.P.dot(self.H.T)) + self.R
        S_inv = np.linalg.inv(self.S)

        # K = P H^ S-1
        self.K = self.P.dot(self.H.T.dot(S_inv))

        # x = x + K y
        self.x = self.x + self.K.dot(y)

        I = np.eye(self.state_dim)
        # P = (I - K H) P
        self.P = (I - self.K.dot(self.H)).dot(self.P)

    def filter(self, z, dt=1.0):
        self.predict()
        self.update(z)
        x = self.H.dot(self.x)
        return x

    @property
    def error(self):
        y = self.z - self.H.dot(self.x)
        S_det = np.linalg.det(self.S)
        S_inv = np.linalg.inv(self.S)
        try:
            err = np.sqrt(S_det) * np.exp(0.5 * y.dot(S_inv).dot(y))
        except OverflowError:
            err = float("inf")
        return err


class Kalman3D(Kalman):
    def __init__(self, P_var=1., Q_var=1e-5, R_var=1e-2):
        super(Kalman3D, self).__init__(
            state_dim=6, meas_dim=3, P_var=P_var, Q_var=Q_var, R_var=R_var)

    def predict(self, dt=1):
        self.F[1, 0] = dt
        self.F[3, 2] = dt
        self.F[5, 4] = dt
        super(Kalman3D, self).predict(dt=dt)


def pose_distance(p1, p2):
    dp  = (p1.pose.position.x - p2.pose.position.x) ** 2
    dp += (p1.pose.position.y - p2.pose.position.y) ** 2
    dp += (p1.pose.position.z - p2.pose.position.z) ** 2

    dt = np.abs((p1.header.stamp - p2.header.stamp).to_sec())
    if dt == 0:
        dt = 1.0
    return np.sqrt(dp) / dt


class PeopleTracker(ConnectionBasedTransport):
    def __init__(self):
        super(PeopleTracker, self).__init__()

        self.fixed_frame_id = rospy.get_param("~fixed_frame_id", "odom_combined")
        self.distance_threshold = rospy.get_param("~distance_threshold", 0.25)
        self.timeout_threshold = rospy.get_param("~timeout_threshold", 5.0)
        self.face_recognition_threshold = rospy.get_param("~face_recognition_threshold", 4300.0)
        self.face_pose_threshold = rospy.get_param("~face_pose_threshold", 0.2)
        self.attention_score_threshold = rospy.get_param("~attention_score_threshold", 30.0)
        self.attention_timeout_threshold = rospy.get_param("~attention_timeout_threshold", 3.0)
        self.unknown_name_prefix = rospy.get_param("~unknown_name_prefix", "unknown")

        self.tfl = tf2_ros.BufferClient("/tf2_buffer_server")
        ok = self.tfl.wait_for_server(timeout=rospy.Duration(10))
        if not ok:
            rospy.logfatal("Error: tf buffer server not found")
            return

        self.people_pub = self.advertise(
            "~output/people", ClassificationResult, queue_size=1)
        self.attention_pub = self.advertise(
            "~output/attention", ClassificationResult, queue_size=1)

    def subscribe(self):
        self.name_counter = defaultdict(int)
        self.people = dict()
        self.last_received = None
        self.attentions = defaultdict(list)

        approximate_sync = rospy.get_param("~approximate_sync", False)
        queue_size = rospy.get_param("~queue_size", 100)
        slop = rospy.get_param("~slop", 0.1)

        self.subscribers = [
            MF.Subscriber("/interaction/face_pose_estimation/output/pose",
                          PoseArray, queue_size=1),
            MF.Subscriber("/interaction/face_recognition/output",
                          FaceArrayStamped, queue_size=1),
        ]

        if approximate_sync:
            sync = MF.ApproximateTimeSynchronizer(self.subscribers,
                                                  queue_size=queue_size,
                                                  slop=slop)
        else:
            sync = MF.TimeSynchronizer(self.subscribers,
                                       queue_size=queue_size)
        sync.registerCallback(self.msg_callback)
        rospy.loginfo("subscribed")

    def unsubscribe(self):
        for s in self.subscribers:
            s.unregister()
        rospy.loginfo("unsubscribed")

    def msg_callback(self, poses, faces):
        # pop old poses
        stamp = poses.header.stamp
        if self.last_received and self.last_received < stamp:
            new_poses = dict()
            for n, p in self.people.items():
                if (stamp - p.header.stamp).to_sec() < self.timeout_threshold:
                    new_poses[n] = p
                else:
                    rospy.logdebug("popped: %s" % n)
            self.people = new_poses

        people_msg = ClassificationResult(header=poses.header)
        people_msg.classifier = "people_tracker"

        attention_msg = ClassificationResult(header=poses.header)
        attention_msg.classifier = "attention_estimator"

        for pose, face in zip(poses.poses, faces.faces):
            rot = pose.orientation
            rot = T.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))
            pose = PoseStamped(header=poses.header, pose=pose)

            # ensure transformation is from fixed frame
            if not pose.header.frame_id != self.fixed_frame_id:
                try:
                    pose = self.tfl.transform(pose, self.fixed_frame_id)
                except:
                    continue

            # find near poses
            dups = list()
            for n, p in self.people.items():
                d = pose_distance(pose, p)
                if d < self.distance_threshold:
                    dups.append((n, p))
                    del self.people[n]

            # merge into one name
            if face.label:
                name = face.label
                proba = (self.face_recognition_threshold - face.confidence) / self.face_recognition_threshold
                rospy.logdebug("%s -> %s (label)" % ([n[0] for n in dups], name))
            elif dups:
                known_names = filter(lambda n: not n[0].startswith(self.unknown_name_prefix), dups)
                if known_names:
                    name = known_names[0][0]
                else:
                    name = dups[0][0]
                proba = 0.5
                rospy.logdebug("%s -> %s" % ([n[0] for n in dups], name))
            else:
                name = self.generate_name()
                rospy.logdebug("None -> %s (new)" % name)
                proba = 0.5

            # fill class msg
            self.people[name] = pose
            people_msg.label_names.append(name)
            people_msg.label_proba.append(proba)

            # count attention
            rot_dist = angles.shortest_angular_distance(rot[0], -np.pi)
            if rot_dist < self.face_pose_threshold:
                self.attentions[name] += [stamp]
            attentions = filter(
                lambda t: (stamp-t).to_sec()<=self.attention_timeout_threshold,
                sorted(self.attentions[name]))
            self.attentions[name] = attentions
            if len(attentions) < 2:
                score = 0.0
            else:
                score = 1.0 * len(attentions) / self.attention_score_threshold
                rospy.logdebug("attention: %f" % score)
            score = min(1.0, max(0.0, score))
            attention_msg.label_names.append(name)
            attention_msg.label_proba.append(score)

        people_msg.target_names = list(set(people_msg.label_names))
        attention_msg.target_names = list(set(attention_msg.label_names))

        self.people_pub.publish(people_msg)
        self.attention_pub.publish(attention_msg)
        self.last_received = stamp

    def generate_name(self, base=None):
        if base is None:
            base = self.unknown_name_prefix
        i = self.name_counter[base]
        self.name_counter[base] += 1
        return base + str(i)


if __name__ == '__main__':
    rospy.init_node("people_tracker")
    t = PeopleTracker()
    rospy.spin()