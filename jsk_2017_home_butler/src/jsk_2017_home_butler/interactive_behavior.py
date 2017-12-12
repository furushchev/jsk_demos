#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import actionlib
import message_filters as MF
import operator
import os
from random import randint
import rospkg
import rospy
from textblob import TextBlob
import tf2_ros
import yaml

from libpgm.discretebayesiannetwork import DiscreteBayesianNetwork
from libpgm.graphskeleton import GraphSkeleton
from libpgm.nodedata import NodeData
from libpgm.tablecpdfactorization import TableCPDFactorization

from jsk_2017_home_butler.utilities import SpeechMixin

from geometry_msgs.msg import PoseStamped, PoseArray
from jsk_recognition_msgs.msg import ClassificationResult
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse


PKGDIR = rospkg.RosPack().get_path("jsk_2017_home_butler")


class InferenceInterface(object):
    def __init__(self):
        graph_path = rospy.get_param(
            "~graph_path", os.path.join(PKGDIR, "data", "graph.yaml"))
        self.load(graph_path)

    def load(self, path):
        with open(path, "r") as f:
            data = yaml.load(f)

        # network
        skel = GraphSkeleton()
        nd = NodeData()
        skel.V = data["V"]
        skel.E = [[e["from"], e["to"]] for e in data["E"]]
        nd.Vdata = data["Vdata"]
        skel.toporder()
        self.network = DiscreteBayesianNetwork(skel, nd)

        # annotators
        self.annotators = data["annotators"]

    def infer(self, evidence, key=None):
        if key is None:
            key = list(set(self.network.V) - set(evidence.keys()))
        elif not isinstance(key, list):
            key = [key]
        query = {k: self.network.Vdata[k]["vals"] for k in key}
        ret = dict()
        for k, v in query.items():
            fn = TableCPDFactorization(self.network)
            cpd = fn.condprobve({k: v}, evidence)
            ret[k] = {v: p for v, p in zip(v, cpd.vals)}
        return ret

    def det(self, prob, key="object"):
        p = sorted(prob[key].iteritems(), key=operator.itemgetter(1), reverse=True)
        L = p[0][1] / p[1][1]
        return L, p[0][0]

    def search(self, evidence, key, threshold):
        nodes = list(set(self.network.V) - set(evidence.keys()) - set([key]))
        nfound = {}
        nhint = {}
        for n in nodes:
            vals = self.network.Vdata[n]["vals"]
            vfound = 0
            hint = None
            for v in vals:
                e = evidence.copy()
                e.update({n: v})
                inferred = self.infer(evidence, key=key)
                L, _ = self.det(inferred, key=key)
                if L > threshold:
                    vfound += 1
                    hint = v
            nfound.update({n: vfound})
            nhint.update({n: hint})

        next_node, n = max(nfound.iteritems(), key=operator.itemgetter(1))
        if n == 1:
            return next_node, nhint[next_node]
        else:
            return next_node, None

    def parse(self, sentence, key=None):
        if key is None:
            variables = {k: v["vals"] for k, v in self.network.Vdata.items()}
        else:
            variables = {key: self.network.Vdata[key]["vals"]}
        data = dict()
        for w in TextBlob(sentence).words:
            w = w.singularize()
            for v, vs in variables.items():
                if w.lower() in vs:
                    data.update({v: w.lower()})
                    break
                elif w.lemma.lower() in vs:
                    data.update({v: w.lemma.lower()})
                    break
        return data


class AttentionEstimation(object):
    def __init__(self):
        self.approximate_sync = False
        self.queue_size = 100
        self.slop = 0.1
        self.threshold = 0.5
        self.sub_counter = 0

    def subscribe(self):
        self.point_head_ac = actionlib.SimpleActionClient(
            "/head_traj_controller/point_head_action", PointHeadAction)
        if not self.point_head_ac.wait_for_server(timeout=rospy.Duration(3.0)):
            rospy.logwarn("/head_traj_controller/point_head_action is not initialized")
            self.point_head_ac = None
        self.tfl = tf2_ros.BufferClient("/tf2_buffer_server")
        ok = self.tfl.wait_for_server(timeout=rospy.Duration(10))
        if not ok:
            rospy.logfatal("Error: tf buffer server not found")
            return
        self.subscribers = [
            MF.Subscriber("/interaction/face_pose_estimation/output/pose",
                          PoseArray, queue_size=1),
            MF.Subscriber("/interaction/people_tracker/output/attention",
                          ClassificationResult, queue_size=1),
        ]

        if self.approximate_sync:
            sync = MF.ApproximateTimeSynchronizer(self.subscribers,
                                                  queue_size=self.queue_size,
                                                  slop=self.slop)
        else:
            sync = MF.TimeSynchronizer(self.subscribers,
                                       queue_size=self.queue_size)
        sync.registerCallback(self.callback)
        rospy.loginfo("subscribed")

    def unsubscribe(self):
        if self.point_head_ac is not None:
            del self.point_head_ac
            self.point_head_ac = None
        for s in self.subscribers:
            s.unregister()
        rospy.loginfo("unsubscribed")

    def __enter__(self):
        if self.sub_counter == 0:
            self.subscribe()
        self.sub_counter += 1

    def __exit__(self, type, value, traceback):
        self.sub_counter -= 1
        if self.sub_counter == 0:
            self.unsubscribe()

    def callback(self, poses, cls):
        try:
            max_proba = max(cls.label_proba)
        except:
            return
        rospy.loginfo("max_proba: %s" % max_proba)
        if max_proba < self.threshold:
            return

        i = cls.label_proba.index(max_proba)
        pose = poses.poses[i]
        name = cls.label_names[i]
        rospy.loginfo("i: %d, name: %s" % (i, name))
        self.person = (name, PoseStamped(header=poses.header, pose=pose))

    def find_person(self, duration=10, who=None):
        self.person = None, None
        with self:
            rate = rospy.Rate(10)
            for i in range(int(duration * 10)):
                name, pose = self.person
                if name:
                    break
                rate.sleep()
        return self.person

    def look_at(self, pose, wait=False):
        assert isinstance(pose, PoseStamped), "%s != PoseStamped" % type(pose)
        with self:
            goal = PointHeadGoal()
            goal.target.header.frame_id = pose.header.frame_id
            goal.target.header.stamp = rospy.Time.now()
            goal.target.point = pose.pose.position
            goal.pointing_axis.x = 1
            goal.pointing_frame = "high_def_frame"
            goal.min_duration = rospy.Duration(1.0)
            goal.max_velocity = 1.0
            if self.point_head_ac is None:
                rospy.logwarn("point head action goal was not sent")
                return
            self.point_head_ac.send_goal(goal)
            if wait:
                self.point_head_ac.wait_for_result(timeout=rospy.Duration(3))

    def look_around(self, wait=False):
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "high_def_frame"
        ps.pose.position.x = 1.0
        ps.pose.position.y = 0.7 * (randint(0, 1) - 0.5)
        self.look_at(ps, wait=wait)

    def wait_for_attention(self, timeout=0):
        start_time = rospy.Time.now()
        r = rospy.Rate(0.5)
        name, pose = None, None
        with self:
            while True:
                if rospy.is_shutdown():
                    break
                elif timeout > 0 and (rospy.Time.now() - start_time).to_sec() > timeout:
                    rospy.logwarn("Timedout")
                    break

                name, pose = self.find_person(duration=10)
                if name:
                    rospy.loginfo("Found %s" % name)
                    self.look_at(pose)
                    break
                r.sleep()
        return name, pose


class InteractiveBehavior(SpeechMixin):
    def __init__(self, iface, atten):
        super(InteractiveBehavior, self).__init__()
        self.iface = iface
        self.atten = atten

        self.enabled = False
        self.srv = rospy.Service("~enable", SetBool, self.enable)
        self.pub_state = rospy.Publisher("~enabled", Bool, queue_size=1, latch=True)

    def enable(self, data):
        if isinstance(data, bool):
            self.enabled = data
        elif isinstance(data, Bool):
            self.enabled = data.data
        else:
            ValueError("data must be bool or Bool")
        self.pub_state.publish(Bool(data=self.enabled))
        msg = "enabled" if self.enabled else "disabled"
        rospy.loginfo("Interactive behavior %s" % msg)
        return SetBoolResponse(success=True, message=msg)

    def clarify(self, key="object", threshold=1.5):
        evidence = {}
        while not rospy.is_shutdown():
            # infer network
            inferred = self.iface.infer(evidence, key=key)
            L, det = self.iface.det(inferred, key=key)

            # check confidence
            if L > threshold:
                return {key: det}

            # ask user
            node, hint = self.iface.search(evidence, key, threshold)
            if hint:
                query = "Is %s %s?" % (node, hint)
            else:
                query = "What is %s?" % (node)
            answer = self.ask(query)
            new_evidence = self.iface.parse(answer, key=node)

            # update evidence
            evidence.update(new_evidence)
            print "new evidence:", new_evidence

    def listen_task(self, name):
        if name == "someone":
            query = "Hi! May I help you?"
        else:
            query = "Hi, %s! May I help you?" % name
        answer = self.ask(query)
        evidence = self.iface.parse(answer)
        if not evidence:
            for i in range(3):
                self.say("Sorry, I cannot hear you.")
                answer = self.ask("Could you repeat please?")
                evidence = self.iface.parse(answer)
                if evidence: break
        if evidence:
            self.say("Let me see.")
        else:
            self.say("Sorry, I cannot hear you.")
            self.say("Please do it yourself!")
        return evidence

    def run(self):
        self.enable(True)

        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            r.sleep()
            if not self.enabled:
                continue

            # wait for attention
            name, pose = self.atten.wait_for_attention()
            if not name:
                continue

            # interact
            evidence = self.listen_task(name)

            # spawn task
            rospy.loginfo("evidence: %s" % evidence)

if __name__ == '__main__':
    rospy.init_node("interactive_behavior")
    iface = InferenceInterface()
    atten = AttentionEstimation()
    n = InteractiveBehavior(iface, atten)
    # evidence = {"face": "furushchev"}
    # inferred = n.infer(evidence)
    # print "evidence:", evidence
    # print "inferred:", inferred

    # d = n.search(evidence={}, key="object")
    # print "search:", d

    # print n.generate({"color": "red"}, key="object", threshold=1.5)

    print n.run()
