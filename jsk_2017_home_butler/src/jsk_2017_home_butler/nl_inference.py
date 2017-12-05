#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import copy
import operator
import os
import rospkg
import rospy
from textblob import TextBlob
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
        self.load_from_yaml(graph_path)

        self.threshold = 0.7
        self.replace_operator = True

    def load_from_yaml(self, path):
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

    def parse_nl(self, sentence):
        variables = {k: v["vals"] for k, v in self.network.Vdata.items()}
        data = dict()
        for w in TextBlob(sentence).words:
            w = w.singularize()
            for v, vs in variables.items():
                if w.lower() in vs:
                    data.update({v:w.lower()})
                    break
                elif w.lemma.lower() in vs:
                    data.update({v:w.lemma.lower()})
                    break
        return data

    def get_nl(self, evidence, inferred, threshold=0.8):
        nl = str()

        act = self.get_result("action", evidence, inferred, threshold)
        if act:
            nl += " " + act
        obj = self.get_result("object", evidence, inferred, threshold)
        if obj:
            nl += " the " + obj
        else:
            appended_the = False
            for anno in filter(lambda a: a["type"] == "object", self.annotators):
                result = self.get_result(anno["name"], evidence, inferred, threshold)
                if result:
                    if not appended_the:
                        nl += " the"
                        appended_the = True
                    nl += " " + result

            cls = self.get_result("class", evidence, inferred, threshold)
            if cls:
                nl += " " + cls
            else:
                nl += " object"

        face = self.get_result("face", evidence, inferred, threshold)
        if face:
            if self.replace_operator:
                nl += " to me"
            else:
                nl += " to " + face

        return nl

    def infer_sentence(self, sentence, evidence={}):
        evidence.update(self.parse_nl(sentence))
        if not evidence:
            return sentence
        inferred = self.infer(evidence)
        result = self.get_nl(evidence, inferred, self.threshold)
        return str(result)


if __name__ == '__main__':
    import pprint

    n = NaturalLanguageInference()
    # sentence = "I'm furushchev. Coffees, please"
    sentence = "boss, please"

    print sentence
    evidence = n.parse_nl(sentence)
    print "=>", evidence
    inferred = n.infer(evidence)
    print "=>"
    pprint.pprint(inferred)
    result = n.get_nl(evidence, inferred, 0.6)
    print "=>", result
