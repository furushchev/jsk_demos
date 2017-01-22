#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


from jsk_owl_exporter import get_mongo_client, BSONConversion
import pymongo
import rospy
import tf2_ros
from sensor_msgs.msg import JointState

TASK = "pr2-fetch-and-place"
CLIENT = get_mongo_client("localhost/jsk_robot_lifelog/pr1012")

def base_pos(task_id, ft, tt):
    res = CLIENT.find({"_meta.stored_type": "move_base_msgs/MoveBaseActionFeedback",
                       "_meta.task_id": task_id,
                       "_meta.inserted_at": { "$gte": ft, "$lte": tt },
                   }).sort([("_meta.inserted_at", -1)]).limit(1)
    if res.count() > 0:
        return res.next()
    else:
        return None

def base_traj(task, dest):
    res = CLIENT.find({"_meta.task_name": TASK,
                       "_meta.task_id": task,
                       "name": "move-to",
                       "args": [dest],
                       "_meta.stored_type": "jsk_robot_startup/ActionEvent",
              }).sort([("$natural", pymongo.ASCENDING)])
    start = None
    end = None
    for d in res:
        task_id = d["_meta"]["task_id"]
        if d["status"] == "START":
            start = d["_meta"]["inserted_at"]
        else:
            end = d["_meta"]["inserted_at"]
    res = CLIENT.find({"_meta.stored_type": "move_base_msgs/MoveBaseActionFeedback",
                       "_meta.task_id": task_id,
                       "_meta.inserted_at": { "$gte": start, "$lte": end },
                   }).sort([("_meta.inserted_at", 1)])
    lines = []
    for d in res:
        pose = d["feedback"]["base_position"]["pose"]["position"]
        lines.append("\t".join([str(pose["x"]), str(pose["y"])]))
    return lines

def cook_traj_stats():
    res = CLIENT.find({"_meta.task_name": TASK,
                       "name": "move-to",
                       "args": ["cook"],
                       "_meta.stored_type": "jsk_robot_startup/ActionEvent",
              })
    ids = []
    for d in res:
        ids.append(d["_meta"]["task_id"])
    with open("cook_base_traj.csv", "w") as f:
        for t in ids:
            traj = base_traj(t, "cook")
            if len(traj) == 0:
                continue
            for l in traj:
                f.write("%s\n" % l)
            f.write("\n\n")

def pick_arm_traj(task):
    rospy.init_node("pick_pose")
    tf_buffer = tf2_ros.Buffer(rospy.Duration(5.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    js_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)

    res = CLIENT.find({"_meta.task_name": TASK,
                       "_meta.task_id": task,
                       "name": "pick",
                       "_meta.stored_type": "jsk_robot_startup/ActionEvent",
              }).sort([("$natural", pymongo.ASCENDING)])
    start = None
    end = None
    for d in res:
        if d["status"] == "START":
            start = d["_meta"]["inserted_at"]
        else:
            end = d["_meta"]["inserted_at"]
    res = CLIENT.find({"_meta.stored_type": "control_msgs/FollowJointTrajectoryActionFeedback",
                       "_meta.task_id": task,
                       "_meta.inserted_at": { "$gte": start, "$lte": end },
                   }).sort([("_meta.inserted_at", 1)])
    cds = []
    for d in res:
        fb = d["feedback"]
        m = JointState()
        stamp = rospy.Time.now()
        stamp.secs = fb["header"]["stamp"]["secs"]
        stamp.nsecs = fb["header"]["stamp"]["nsecs"]
        m.header.stamp = stamp
        m.header.seq = fb["header"]["seq"]
        m.name = fb["joint_names"]
        m.position = fb["actual"]["positions"]
        m.velocity = fb["actual"]["velocities"]
        js_pub.publish(m)
        rospy.sleep(0.1)
        try:
            end_coords = tf_buffer.lookup_transform("base_footprint",
                                                    "r_wrist_roll_link",
                                                    stamp,
                                                    rospy.Duration(0.05))
            if len(cds) == 0:
                cds.append(end_coords)
            else:
                last_cds = cds[-1]
                if not (end_coords.header.stamp - last_cds.header.stamp).is_zero():
                    cds.append(end_coords)
        except:
            pass
    return cds

def pick_arm_traj_stats():
    res = CLIENT.find({"_meta.task_name": TASK,
                       "name": "pick",
                       "args": ["cup", "rarm"],
                       "_meta.stored_type": "jsk_robot_startup/ActionEvent",
              })
    ids = []
    for d in res:
        ids.append(d["_meta"]["task_id"])
    with open("pick_cup_arm.csv", "w") as f:
        for t in ids:
            cds = pick_arm_traj(t)
            if len(cds) == 0:
                continue
            lines = []
            for m in cds:
                lines.append("\t".join([str(m.transform.translation.x),
                                        str(m.transform.translation.y),
                                        str(m.transform.translation.z)]) + "\n")
            f.writelines(lines)
            f.write("\n\n")

        
def goal_stats():
    task_result = {}
    res = CLIENT.find({"_meta.task_name": TASK,
                       "name": "pick",
                       "_meta.stored_type": "jsk_robot_startup/ActionEvent",
              }).sort([("$natural", pymongo.ASCENDING)])
    for d in res:
        if d["status"] == "SUCCESS":
            task_result[d["_meta"]["task_id"]] = True
        elif d["status"] == "FAILURE":
            task_result[d["_meta"]["task_id"]] = False

    res = CLIENT.find({"_meta.task_name": TASK,
                       "name": "move-to",
                       "args": ["table"],
                       "_meta.stored_type": "jsk_robot_startup/ActionEvent",
              }).sort([("$natural", pymongo.ASCENDING)])
    ft = {}
    success = []
    failure = []
    unknown = []
    for d in res:
        task_id = d["_meta"]["task_id"]
        if d["status"] == "START":
            ft[task_id] = d["_meta"]["inserted_at"]
        else:
            base = base_pos(task_id, ft[task_id], d["_meta"]["inserted_at"])
            if base is not None:
                pose = base["feedback"]["base_position"]["pose"]["position"]
                tres = 0
                if task_id in task_result.keys():
                    tres = 1 if task_result[task_id] else 2
                if tres == 0:
                    unknown.append("\t".join([str(pose["x"]), str(pose["y"])])+"\n")
                elif tres == 1:
                    success.append("\t".join([str(pose["x"]), str(pose["y"])])+"\n")
                elif tres == 2:
                    failure.append("\t".join([str(pose["x"]), str(pose["y"])])+"\n")

    with open("base.csv", "w") as f:
        f.writelines(success)
        f.write("\n\n")
        f.writelines(failure)
        f.write("\n\n")
        f.writelines(unknown)
        f.write("\n\n")


if __name__ == '__main__':
#    goal_stats()
#    cook_traj_stats()
    pick_arm_traj_stats()

