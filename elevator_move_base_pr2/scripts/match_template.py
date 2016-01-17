#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from rospkg import RosPack

from sensor_msgs.msg import Image
from elevator_move_base_pr2.msg import MatchedTemplates
from elevator_move_base_pr2.srv import ChangeMatchingThreshold, ChangeMatchingThresholdResponse

rospack = RosPack()

class NamedDict(dict):
    def __getattr__(self, name):
        return self.__getitem__(name)
    def __setattr__(self, name, value):
        self.__setitem__(name, value)

class OnDemandSubscribeListener(rospy.SubscribeListener):
    def add_callback(self, topic_name, topic_class, callback_function):
        self.sub = None
        self.sub_topic_name = topic_name
        self.sub_topic_class = topic_class
        self.callback_function = callback_function
    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        if self.sub is None:
            self.sub = rospy.Subscriber(self.sub_topic_name, self.sub_topic_class,
                                        self.callback_function, queue_size=1)
    def peer_unsubscribe(self, topic_name, num_peers):
        if num_peers == 0:
            self.sub.unregister()
            self.sub = None

def resolve_ros_path(path):
    if "package://" in path:
        plist = path.split('/')
        pkg_dir = rospack.get_path(plist[2])
        return "/".join([pkg_dir] + plist[3:])
    return path

def to_3ch(a):
    return np.dstack((a,a,a))

class MatchTemplateNode(object):
    def __init__(self):
        self.cv_bridge = CvBridge()
        self.templates = self.read_templates_from_params()
        listener = OnDemandSubscribeListener()
        listener.add_callback("~image", Image, self.image_callback)
        self.pub = rospy.Publisher("~result", MatchedTemplates, subscriber_listener=listener)
        self.debug_pub = rospy.Publisher("~debug_image", Image)
        self.srv_thre = rospy.Service("~change_threshold", ChangeMatchingThreshold, self.update_threshold_callback)

    def read_templates_from_params(self):
        return [NamedDict(typename=typename,
                          image=cv2.imread(resolve_ros_path(rospy.get_param('~template/' + typename + '/path')),
                                           cv2.IMREAD_GRAYSCALE),
                          threshold=rospy.get_param('~template/' + typename + '/thre'),
                          name=rospy.get_param('~template/' + typename + '/name'),
                          method=self.template_match_method(rospy.get_param('~template/' + typename + '/method', ''))) for typename in rospy.get_param('~template_list').split()]

    def template_match_method(self, s):
        if s == 'CCORR':
            method = cv2.TM_CCORR_NORMED
        elif s == 'CCOEFF':
            method = cv2.TM_CCOEFF_NORMED
        else:
            method = cv2.TM_SQDIFF_NORMED
        return method

    def update_threshold_callback(self, req):
        res = ChangeMatchingThresholdResponse()
        try:
            idx = [t.name for t in self.templates].index(req.name)
            old_thre = self.templates[idx].threshold
            self.templates[idx].threshold = req.threshold
            rospy.loginfo("changed %s threshold %f -> %f", req.name, old_thre, self.templates[idx].threshold)
            res.ok = True
        except Exception as e:
            rospy.logerr("Error: cannot change threshold of %s: %s", req.name, e)
            res.ok = False
        finally:
            return res

    def image_callback(self, imgmsg):
        try:
            image = self.cv_bridge.imgmsg_to_cv2(imgmsg, "mono8")
        except Exception as e:
            rospy.logerr('Error: cannot convert image to mono8: %s', e)
            return
        results = self.match_templates(image)
        self.publish_result(results)
        self.publish_debug_image(image, results)

    def match_templates(self, cam_image):
        return [self.match_template(cam_image, t.name, t.image, t.threshold, t.method) for t in self.templates]

    def match_template(self, cam_image, name, image, threshold, method):
        res = cv2.matchTemplate(cam_image, image, method)
        minval, maxval, minloc, maxloc = cv2.minMaxLoc(res)

        if method == cv2.TM_SQDIFF_NORMED:
            value = minval
            location = minloc
            found = value < threshold
        else:
            value = maxval
            location = maxloc
            found = threshold < value

        rospy.loginfo("%s: %f < %f, %s", name, value, threshold, found)
        return NamedDict(name=name,
                         value=value,
                         location=location,
                         threshold=threshold,
                         found=found)

    def publish_result(self, results):
        msg = MatchedTemplates()
        for r in results:
            if r.found:
                msg.names.append(r.name)
                msg.values.append(r.value)
                msg.thresholds.append(r.threshold)
        self.pub.publish(msg)

    def publish_debug_image(self, cam_image, results):
        width = max(cam_image.shape[1], sum([i.image.shape[1] for i in self.templates]))
        height = cam_image.shape[0] + max([i.image.shape[0] for i in self.templates])
        debug_img = np.zeros((height, width, 3), dtype=np.uint8)
        debug_img[height-cam_image.shape[0]:height,0:width] = to_3ch(cam_image)
        cur_x = 0
        for t in self.templates:
            debug_img[0:t.image.shape[0],cur_x:cur_x+t.image.shape[1]] = to_3ch(t.image)
            res = [r for r in results if r.name == t.name][0]
            cv2.putText(debug_img, t.name, (cur_x,t.image.shape[0]-42),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.CV_AA)
            cv2.putText(debug_img, "%7.5f" % res.value, (cur_x,t.image.shape[0]-24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.CV_AA)
            cv2.putText(debug_img, "%7.5f" % res.threshold, (cur_x,t.image.shape[0]-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.CV_AA)
            if res.found:
                cv2.rectangle(debug_img, (cur_x,0), (cur_x + t.image.shape[1], t.image.shape[0]), (255,255,255), 9)
            cur_x += t.image.shape[1]

        imgmsg = self.cv_bridge.cv2_to_imgmsg(debug_img, "bgr8")
        self.debug_pub.publish(imgmsg)

if __name__ == '__main__':
    rospy.init_node("match_template")
    n = MatchTemplateNode()
    rospy.spin()
