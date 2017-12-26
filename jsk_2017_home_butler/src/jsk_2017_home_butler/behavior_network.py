#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>

import yaml
import pandas as pd
import pymc
from dstk.pymc_utils import make_bernoulli, make_categorical, cartesian_child
from dstk.imputation import BayesNetImputer, StringFeatureEncoder


class AnnotationNetworkImputer(BayesNetImputer):
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


class BehaviorNetworkImputer(AnnotationImputer):
    annotators = ["color", "shape", "volume", "face"]


class BehaviorNetwork(object):
    def __init__(self, n=10000, burn=8000, thin=10):
        super(BehaviorNetwork, self).__init__()
        self.encoder = StringFeatureEncoder(missing_marker="__MISSING__")
        self.network = None
        self.n = n
        self.burn = burn
        self.thin = thin

    def load(self, in_path):
        with open(in_path, "r") as f:
            data = yaml.load(f)["data"]
        data = data.fillna("__MISSING__")
        self.data = self.encoder.fit(data).transform(data).replace(-1, 0).subtract(1)
        return self

    def learn(self, method="MCMC"):
        self.network = BehaviorNetworkImputer(method=method).fit(self.data)
        return self

    def infer(self, **evidence):
        data = pd.DataFrame(columns=self.network.nodes.keys())
        data = data.append(evidence, ignore_index=True)

        sampler = pymc.MCMC(self.network)
        sampler.sample(iter=self.n, burn=self.burn, thin=self.thin)
        
