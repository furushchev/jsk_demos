#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

from jsk_topic_tools import ConnectionBasedTransport
import message_filters as MF
import numpy as np
import rospy
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
        self.distance_threshold = rospy.get_param("~distance_threshold", 0.1)
        self.timeout_threshold = rospy.get_param("~timeout_threshold", 3.0)
        self.face_recognition_threshold = rospy.get_param("~face_recognition_threshold", 4300.0)

        self.tfl = tf2_ros.BufferClient("/tf2_buffer_server")
        ok = self.tfl.wait_for_server(timeout=rospy.Duration(10))
        if not ok:
            rospy.logfatal("Error: tf buffer server not found")
            return

        self.class_pub = self.advertise(
            "~output", ClassificationResult, queue_size=1)

    def subscribe(self):
        self.counter = 0
        self.people = dict()
        self.last_received = None

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
        # filter cached poses
        if self.last_received and self.last_received < poses.header.stamp:
            now = poses.header.stamp
            new_poses = dict()
            for n, p in self.people.items():
                if (now - p.header.stamp).to_sec() < self.timeout_threshold:
                    new_poses[n] = p
                else:
                    rospy.logdebug("popped: %s" % n)
            self.people = new_poses

        msg = ClassificationResult(header=poses.header)
        msg.classifier = "people_tracker"

        for pose, face in zip(poses.poses, faces.faces):
            pose = PoseStamped(header=poses.header, pose=pose)
            near_name, dist = self.nearest_person(pose)
            if face.label:
                name = face.label
                proba = (self.face_recognition_threshold - face.confidence) / self.face_recognition_threshold
                if near_name is not None and dist < self.distance_threshold:
                    rospy.logdebug("%s -> %s" % (near_name, name))
                    del self.people[near_name]
            elif near_name is not None and dist < self.distance_threshold:
                name = near_name
                proba = 0.5
                rospy.logdebug("None -> %s" % name)
            else:
                name = self.generate_name()
                rospy.logdebug("None -> %s (new)" % name)
                proba = 0.5
            self.people[name] = pose
            msg.label_names.append(name)
            msg.label_proba.append(proba)
        msg.target_names = list(set(msg.label_names))

        self.class_pub.publish(msg)
        self.last_received = poses.header.stamp

    def generate_name(self):
        s = "unknown%d" % self.counter
        self.counter += 1
        return s

    def nearest_person(self, pose):
        if not pose.header.frame_id != self.fixed_frame_id:
            pose1 = self.tfl.transform(pose, self.fixed_frame_id)
        else:
            pose1 = pose

        min_distance = 1000
        min_person = None, None
        for name, pose2 in self.people.items():
            d = pose_distance(pose1, pose2)
            if d < min_distance:
                min_distance = d
                min_person = (name, d)
        return min_person


if __name__ == '__main__':
    rospy.init_node("people_tracker")
    t = PeopleTracker()
    rospy.spin()
