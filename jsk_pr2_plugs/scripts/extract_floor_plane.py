#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import numpy as np
import message_filters
import rospy
from scipy.spartial import ConvexHull
import tf

from geometry_msgs.msg import PolygonStamped
from jsk_recognition_msgs.msg import (ModelCoefficientsArray, PolygonArray)
from pcl_msgs.msg import ModelCoefficients
from sensor_msgs.msg import PointCloud


class Polygon(PolygonStamped):
    def __init__(self, frame_id):
        super(Polygon, self).__init__()
        self.np_pts = np.array([[p.x,p.y,p.z] for p in self.polygon.points])
        self.tf_listener = tf.TransformListsner()
        pc = PointCloud()
        pc.header = self.header
        pc.points = self.polygon.points
        self.transformed_pts = self.tf_listener.transformPointCloud(frame_id, pc)
    def z_max(self, frame_id):
        if len(self.polygon.points) == 0:
            return None
        pc = PointCloud()
        pc.header = self.header
        pc.points = self.polygon.points
        trans = tf.transformPointCloud(frame_id, pc)
        zmax = trans.points[0].z
        for p in trans.points:
            if zmax < p.z:
                zmax = p.z
        return zmax
    def z_min(self, frame_id):
        if len(self.polygon.points) == 0:
            return None
        pc = PointCloud()
        pc.header = self.header
        pc.points = self.polygon.points
        trans = tf.transformPointCloud(frame_id, pc)
        zmin = trans.points[0].z
        for p in trans.points:
            if zmin > p.z:
                zmin = p.z
        return zmin
    def area(self):
        hull = ConvexHull(self.np_pts)
        return hull.area

class ExtractFloorPlane(object):
    def __init__(self):
        self.queue_size = rospy.get_param("~queue_size", 100)
        self.spin_rate = rospy.Rate(10)
        self.pub_coeff = rospy.Publisher("~output/coefficients", ModelCoefficients)
        self.pub_polygon = rospy.Publisher("~output/polygon", PolygonStamped)
        self.subscribers = []
    def subscribe(self):
        self.subscribers += [
            message_filters.Subscriber("~input/polygons", PolygonArray),
            message_filters.Subscriber("~input/coefficients", ModelCoefficientsArray),
        ]
        ts = message_filters.TimeSynchronizer(self.subscribers, self.queue_size)
        ts.reigsterCallback(self, callback)
    def unsubscribe(self):
        for i, s in enumerate(self.subscribers):
            s.sub.unregister()
            del self.subscribers[i]
    def spin(self):
        while not rospy.is_shutdown():
            if len(self.subscribers) > 0 and self.pub_coeff.get_num_connections() == 0 and self.pub_polygon.get_num_connections():
                self.unsubscribe()
            elif len(self.subscribers) == 0 and (self.pub_coeff.get_num_connections() > 0 or self.pub_polygon.get_num_connections()):
                self.subscribe()
            self.spin_rate.sleep()

    def callback(self, polygons, coeffs):
        if len(polygons.polygons) == 0:
            return

        # filter with normal vector
        nvecs = []
        for c in coeffs.coefficients:
            pts = np.array()

        # filter with area
        area = []
        for poly in polygons.polygons:
            pts = np.array([[p.x,p.y,p.z] for p in poly.points])
            area.append(ConvexHull(pts))


if __name__ == '__main__':
    rospy.init_node("extract_floor_plane")
    node = ExtractFloorPlane()
    node.spin()
