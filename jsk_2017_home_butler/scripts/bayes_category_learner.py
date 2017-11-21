#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import csv
import os
import pprint
import rospkg
import rospy

from libpgm.discretebayesiannetwork import DiscreteBayesianNetwork
from libpgm.graphskeleton import GraphSkeleton
from libpgm.nodedata import NodeData
from libpgm.pgmlearner import PGMLearner
from libpgm.tablecpdfactorization import TableCPDFactorization


PKGDIR = rospkg.RosPack().get_path("jsk_2017_home_butler")


class BayesCategoryLearner(object):
    def __init__(self):
        super(BayesCategoryLearner, self).__init__()
        class_path = os.path.join(PKGDIR, "data", "categories")
        self.classes = dict()
        for f in os.listdir(class_path):
            if f.endswith(".class"):
                self.classes[f[:-6]] = os.path.join(class_path, f)

    def make_network(self, skel, data):
        skel.toporder()
        bn = DiscreteBayesianNetwork()

        # build structure
        bn.V = skel.V
        bn.E = skel.E
        bn.Vdata = dict()
        for vertex in bn.V:
            bn.Vdata[vertex] = dict()
            bn.Vdata[vertex]["children"] = skel.getchildren(vertex)
            bn.Vdata[vertex]["parents"] = skel.getparents(vertex)

            bn.Vdata[vertex]["vals"] = []
            if (bn.Vdata[vertex]["parents"] == []):
                bn.Vdata[vertex]["cprob"] = []
            else:
                bn.Vdata[vertex]["cprob"] = dict()

            bn.Vdata[vertex]["numoutcomes"] = 0

        for sample in data:
            for vertex in bn.V:
                if (vertex in sample and sample[vertex] not in bn.Vdata[vertex]["vals"]):
                    bn.Vdata[vertex]["vals"].append(sample[vertex])
                    bn.Vdata[vertex]["numoutcomes"] += 1

        def addlevel(vertex, _dict, key, depth, totaldepth):
            if depth == totaldepth:
                _dict[str(key)] = []
                for _ in range(bn.Vdata[vertex]["numoutcomes"]):
                    _dict[str(key)].append([0, 0])
                return
            else:
                for val in bn.Vdata[bn.Vdata[vertex]["parents"][depth]]["vals"]:
                    ckey = key[:]
                    ckey.append(str(val))
                    addlevel(vertex, _dict, ckey, depth+1, totaldepth)

        # create cpt with zero entites
        for vertex in bn.V:
            if (bn.Vdata[vertex]["parents"]):
                root = bn.Vdata[vertex]["cprob"]
                numparents = len(bn.Vdata[vertex]["parents"])
                addlevel(vertex, root, [], 0, numparents)
            else:
                for _ in range(bn.Vdata[vertex]["numoutcomes"]):
                    bn.Vdata[vertex]["cprob"].append([0, 0])

        # fill out entries with samples
        for sample in data:
            for vertex in bn.V:
                if vertex not in sample: continue

                rindex = bn.Vdata[vertex]["vals"].index(sample[vertex])

                if bn.Vdata[vertex]["parents"]:
                    pvals = [str(sample[t]) for t in bn.Vdata[vertex]["parents"]]
                    lev = bn.Vdata[vertex]["cprob"][str(pvals)]
                else:
                    lev = bn.Vdata[vertex]["cprob"]

                for entry in lev:
                    entry[1] += 1

                lev[rindex][0] += 1

        # standardization
        for vertex in bn.V:
            if not bn.Vdata[vertex]["parents"]:
                bn.Vdata[vertex]["cprob"] = [x[0]/float(x[1]) for x in bn.Vdata[vertex]["cprob"]]
            else:
                for key in bn.Vdata[vertex]["cprob"].keys():
                    try:
                        bn.Vdata[vertex]["cprob"][key] = [x[0]/float(x[1]) for x in bn.Vdata[vertex]["cprob"][key]]
                    except ZeroDivisionError:
                        # default to even distribution if no data points
                        bn.Vdata[vertex]["cprob"][key] = [1/float(bn.Vdata[vertex]["numoutcomes"]) for x in bn.Vdata[vertex]["cprob"][key]]

        return bn

    def learn(self):
        data = list()
        V = set()
        for category, path in self.classes.items():
            with open(path, "r") as f:
                for row in csv.reader(f):
                    c = dict([c.strip().split('=') for c in row[1:]])
                    c["category"] = category
                    data += [c]
                    V |= set(c.keys())
        E = [("category", a) for a in V - set(["category"])]

        skel = GraphSkeleton()
        skel.V, skel.E = list(V), E

        network = self.make_network(skel, data)
        pprint.pprint(network.Vdata)


if __name__ == '__main__':
    rospy.init_node("bayes_category_learner")
    learner = BayesCategoryLearner()
    learner.learn()
