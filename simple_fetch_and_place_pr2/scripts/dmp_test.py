#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


import rospy
from dmp.msg import *
from dmp.srv import *

import numpy as np

dims = 3
dt = 0.1
K = 100
D = 2.0 * np.sqrt(K)
num_bases = 4
pick_cup_traj = []

with open("pick_cup_arm.csv", "r") as f:
    while True:
        line = f.readline()
        if not line.strip():
            break
        pick_cup_traj.append(map(float, line.strip().split()))

msg = DMPTraj()
for i, t in enumerate(pick_cup_traj):
    pt = DMPPoint()
    pt.positions = t
    msg.points.append(pt)
    msg.times.append(dt*i)

rospy.wait_for_service("learn_dmp_from_demo")
lfd = rospy.ServiceProxy("learn_dmp_from_demo", LearnDMPFromDemo)
res = lfd(msg, [K]*dims, [D]*dims, num_bases)
print res

if __name__ == '__main__':
    pass
