#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import os
import pprint
import traceback
import yaml

from libpgm.discretebayesiannetwork import DiscreteBayesianNetwork
from libpgm.graphskeleton import GraphSkeleton
from libpgm.nodedata import NodeData
from libpgm.pgmlearner import PGMLearner
from libpgm.tablecpdfactorization import TableCPDFactorization


def dump_graph(graph, out):
    assert hasattr(graph, "V"), "No node found in graph"
    assert hasattr(graph, "E"), "No edge found in graph"
    assert hasattr(graph, "Vdata"), "No CPT found in graph"
    assert not os.path.exists(out), "Output path already exists"

    d = dict()
    d["V"] = graph.V
    d["E"] = graph.E
    d["Vdata"] = graph.Vdata

    with open(out, "w") as f:
        yaml.dump(d, f)

    print "Generated %s" % out
    return True


def make_network(skel, data):
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

    # TODO: add GETA

    return bn


def learn_graph(data_path):
    assert os.path.exists(data_path), "No data file found"
    with open(data_path, "r") as f:
        try:
            config = yaml.load(f)
            data, meta = config["data"], config["graph"]
        except Exception as e:
            print "failed to load data:", e
            traceback.print_exc()
            return False

    # load information
    try:
        annotators = meta["annotators"]
        print "annotators: %s" % annotators
    except:
        print "no annotators information found"
        print "'info.annotators' is missing"
        return False
    try:
        edges = meta["structure"]
        print "%d edges found" % len(edges)
    except:
        print "no structure information found"
        print "'info.structure' is missing"
        return False

    # aggregate rules

    skel = GraphSkeleton()
    skel.V, skel.E = list(), list()
    skel.V += annotators
    for e in edges:
        skel.V += [e["from"], e["to"]]
    skel.V = list(set(skel.V))
    skel.V.remove("annotators")
    skel.E = list()
    for e in edges:
        if e["from"] == "annotators":
            skel.E += [[a, e["to"]] for a in annotators]
        elif e["to"] == "annotators":
            skel.E += [[e["from"], a] for a in annotators]
        else:
            skel.E += [[e["from"], e["to"]]]

    network = make_network(skel, data)

    return network


if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("data", help="path to input data file")
    p.add_argument("--out", "-o", help="path to output file",
                   default="graph.yaml")

    args = p.parse_args()

    network = learn_graph(args.data)
    dump_graph(network, args.out)
