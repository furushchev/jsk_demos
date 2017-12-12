#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import operator
import os
import rospkg
import rospy
import yaml

from textblob import TextBlob

from libpgm.discretebayesiannetwork import DiscreteBayesianNetwork
from libpgm.graphskeleton import GraphSkeleton
from libpgm.nodedata import NodeData
from libpgm.pgmlearner import PGMLearner
from libpgm.tablecpdfactorization import TableCPDFactorization
from libpgm.sampleaggregator import SampleAggregator

from jsk_2017_home_butler.utilities import SpeechMixin


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

    def parse(self, sentence):
        variables = {k: v["vals"] for k, v in self.network.Vdata.items()}
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


class InteractiveBehavior(SpeechMixin):
    def __init__(self, iface):
        super(InteractiveBehavior, self).__init__()
        self.iface = iface

    def ask(self, query):
        answer = super(InteractiveBehavior, self).ask(query)
        return self.iface.parse(answer)

    def run(self, key="object", threshold=1.5):
        evidence = {}
        while not rospy.is_shutdown():
            inferred = self.iface.infer(evidence, key=key)
            L, det = self.iface.det(inferred, key=key)
            if L > threshold:
                return {key: det}

            node, hint = self.iface.search(evidence, key, threshold)
            if hint:
                query = "Is %s %s?" % (node, hint)
            else:
                query = "What is %s?" % (node)

            new_evidence = self.ask(query)
            evidence.update(new_evidence)
            print "new evidence:", new_evidence


if __name__ == '__main__':
    rospy.init_node("interactive_behavior")
    iface = InferenceInterface()
    n = InteractiveBehavior(iface)
    # evidence = {"face": "furushchev"}
    # inferred = n.infer(evidence)

    # print "evidence:", evidence
    # print "inferred:", inferred

    # d = n.search(evidence={}, key="object")
    # print "search:", d

    # print n.generate({"color": "red"}, key="object", threshold=1.5)

    print n.run()
