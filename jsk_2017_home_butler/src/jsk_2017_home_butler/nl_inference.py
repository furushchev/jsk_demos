#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import copy
import operator
import os
import rospkg
import rospy
import yaml

from libpgm.discretebayesiannetwork import DiscreteBayesianNetwork
from libpgm.graphskeleton import GraphSkeleton
from libpgm.nodedata import NodeData
from libpgm.pgmlearner import PGMLearner
from libpgm.tablecpdfactorization import TableCPDFactorization
from libpgm.sampleaggregator import SampleAggregator


PKGDIR = rospkg.RosPack().get_path("jsk_2017_home_butler")


def argmax(d):
    return max(d.iteritems(), key=operator.itemgetter(1))


class NaturalLanguageInference(object):
    def __init__(self):
        super(NaturalLanguageInference, self).__init__()

        graph_path = rospy.get_param(
            "~graph_path", os.path.join(PKGDIR, "data", "graph.yaml"))
        self.sample_num = rospy.get_param("~sample_num", 10000)
        self.network, self.annotators = self.load_network(graph_path)

    def load_network(self, path):
        with open(path, "r") as f:
            data = yaml.load(f)

        skel = GraphSkeleton()
        nd = NodeData()
        skel.V = data["V"]
        skel.E = [[e["from"], e["to"]] for e in data["E"]]
        nd.Vdata = data["Vdata"]
        skel.toporder()
        return DiscreteBayesianNetwork(skel, nd), data["annotators"]

    def infer(self, evidence):
        var = list(set(self.network.V) - set(evidence.keys()))
        query = {v: self.network.Vdata[v]["vals"] for v in var}
        # print query
        ret = {}
        for k,v in query.items():
            fn = TableCPDFactorization(self.network)
            cpd = fn.condprobve({k:v}, evidence)
            ret[k] = {v:p for v, p in zip(v, cpd.vals)}
        return ret

    def get_result(self, key, evidence, inferred, threshold=0.8):
        if key in evidence:
            return evidence[key]

        candidate = argmax(inferred[key])
        if candidate[1] > threshold:
            return candidate[0]

        return str()

    def get_nl(self, evidence, inferred, threshold=0.8):
        nl = str()
        # action
        act = self.get_result("action", evidence, inferred, threshold)
        if act:
            nl += " " + act
        obj = self.get_result("object", evidence, inferred, threshold)
        if obj:
            nl += " " + obj
        else:
            for anno in filter(lambda a: a["type"] == "object", self.annotators):
                result = self.get_result(anno["name"], evidence, inferred, threshold)
                if result:
                    nl += " " + result

            cls = self.get_result("class", evidence, inferred, threshold)
            if cls:
                nl += " " + cls
            else:
                nl += " object"

        face = self.get_result("face", evidence, inferred, threshold)
        if face:
            nl += " to " + face

        print nl


if __name__ == '__main__':
    evidence = {
        "face": "furushchev",
        "class": "coffee",
    }

    n = NaturalLanguageInference()
    inferred = n.infer(evidence)
    n.get_nl(evidence, inferred, 0.7)

