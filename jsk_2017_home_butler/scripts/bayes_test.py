#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import pprint

from libpgm.discretebayesiannetwork import DiscreteBayesianNetwork
from libpgm.graphskeleton import GraphSkeleton
from libpgm.nodedata import NodeData
from libpgm.pgmlearner import PGMLearner
from libpgm.tablecpdfactorization import TableCPDFactorization


def make_data(l,m,n):
    data = list()
    for _ in range(l):
        data.append({
            "category": "georgia",
            "color": "blue",
            "volume": "medium",
            "shape": "circle",
            })
    data.append({
        "category": "georgia",
        "color": "black",
        "volume": "medium",
        "shape": "circle",
    })
    data.append({
        "category": "georgia",
        "color": "white",
        "volume": "medium",
        "shape": "circle",
    })
    for _ in range(m):
        data.append({
            "category": "wanda",
            "color": "red",
            "volume": "medium",
            "shape": "circle",
        })
    data.append({
        "category": "wanda",
        "color": "black",
        "volume": "medium",
        "shape": "circle",
    })
    for _ in range(n):
        data.append({
            "category": "iemon",
            "color": "green",
            "volume": "medium",
            "shape": "rectangle",
        })
    data.append({
        "category": "iemon",
        "color": "white",
        "volume": "big",
        "shape": "rectangle",
    })

    return data


def make_graph():
    categories = ["georgia", "iemon", "wanda"]
    recognizers = {
        "color": ["red", "green", "blue", "black", "white"],
        "volume": ["small", "medium", "big"],
        "shape": ["circle", "rectangle"],
    }

    skel = GraphSkeleton()
    skel.V = ["category"] + recognizers.keys()
    skel.E = [["category", s] for s in recognizers.keys()]
    skel.toporder()

    samples = make_data(10, 10, 10)

    learner = PGMLearner()
    data = learner.discrete_mle_estimateparams(skel, samples)

    network = DiscreteBayesianNetwork(skel, data)

    pprint.pprint(network.Vdata)

    cpd = TableCPDFactorization(network)
    proba = cpd.condprobve(
        query={"category": ["georgia", "wanda", "iemon"]},
        evidence={"color": "blue"})
    print proba.scope, proba.vals

if __name__ == '__main__':
    make_graph()
