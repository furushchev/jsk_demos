#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

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


class InferNetwork(object):
    def __init__(self):
        super(InferNetwork, self).__init__()

        graph_path = rospy.get_param(
            "~graph_path", os.path.join(PKGDIR, "data", "graph.yaml"))
        self.sample_num = rospy.get_param("~sample_num", 10000)
        self.network = self.load_network(graph_path)

    def load_network(self, path):
        with open(path, "r") as f:
            data = yaml.load(f)

        skel = GraphSkeleton()
        nd = NodeData()
        skel.V, skel.E, nd.Vdata = data["V"], data["E"], data["Vdata"]
        skel.toporder()
        return DiscreteBayesianNetwork(skel, nd)

    def infer(self, evidence):
        var = list(set(self.network.V) - set(evidence.keys()))
        query = {v: self.network.Vdata[v]["vals"] for v in var}

        cpd = TableCPDFactorization(self.network)
        agg = SampleAggregator()

        rospy.loginfo("Inferring...")
        burn_in = int(self.sample_num * 0.1)
        result = agg.aggregate(cpd.gibbssample(evidence, self.sample_num)[burn_in:])

        return result


if __name__ == '__main__':
    n = InferNetwork()
    res = n.infer({"face": "furushchev",
                   "class": "tea"})
    print res
