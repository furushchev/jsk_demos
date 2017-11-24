#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import yaml
import pandas as pd
import pymc
import pprint
from dstk.pymc_utils import make_bernoulli, make_categorical, cartesian_child
from dstk.imputation.encoders import MasterExploder
from dstk.imputation import BayesNetImputer, StringFeatureEncoder

class AnnotationImputer(BayesNetImputer):
    annotators = list()
    def construct_net(self, df):
        def get_level(d, n):
            if d[n][d[n] == -1].count() > 0:
                return d[n].nunique() - 1
            else:
                return d[n].nunique()
        if get_level(df, "action") == 2:
            act = make_bernoulli("action", value=df["action"])
        else:
            act = make_categorical('action', levels=get_level(df, "action"), value=df["action"])
        cls = cartesian_child('class', parents=[act], levels=get_level(df, "class"), value=df["class"])
        obj = cartesian_child('object', parents=[cls], levels=get_level(df, "object"), value=df["object"])

        self.nodes = {"action": act, "class": cls, "object": obj}

        # annotators
        for anno in self.annotators:
            n = cartesian_child(anno,  parents=[obj], levels=get_level(df, anno), value=df[anno])
            self.nodes[anno] = n

        return pymc.Model(self.nodes.values())


class HomeButlerImputer(AnnotationImputer):
    annotators = ["color", "shape", "volume", "face"]


def load_data(in_path):
    with open(in_path, "r") as f:
        data = yaml.load(f)["data"]
    df = pd.DataFrame(data)
    return df

def learn():
    data = load_data("../data/annotations.yaml")
    data = data.fillna("MISSING")
    print data

    encoder = StringFeatureEncoder(missing_marker="MISSING")
    t = encoder.fit(data).transform(data).replace(-1, 0).subtract(1)
    network = HomeButlerImputer(method="MCMC")
    return network, encoder

def infer(network, evidence, encoder):
    e = network.fit_transform(t)
    print encoder.inverse_transform(e.add(1).replace(0, -1))

    sampler = pymc.MCMC(network)
    sampler.sample(iter=10000, burn=8000, thin=10)
    pprint.pprint(sampler.stats())
    print dir(sampler)


if __name__ == '__main__':
    learn()
