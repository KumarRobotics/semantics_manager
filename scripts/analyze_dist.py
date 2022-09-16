#!/usr/bin/env python3

import numpy as np
from tqdm import tqdm
import argparse

import rosbag

class DistAnalyzer:
    def __init__(self, bag_path, n_robots, real):
        self.real_ = real
        self.robot_list_ = {}
        if real:
            self.robot_list_["io"] = 0
            self.robot_list_["europa"] = 1
            self.robot_list_["callisto"] = 2
        else:
            for i in range(n_robots):
                robot_name = f"husky{i+1}"
                self.robot_list_[robot_name] = i

        self.topic_list_ = []
        for robot_name in self.robot_list_:
            self.topic_list_.append(f"/{robot_name}/top_down_render/pose_est")
            self.topic_list_.append(f"/{robot_name}/jackal_teleop/is_auto")

        self.last_poses_ = [None for r in range(n_robots)]
        self.is_auto_ = [False for r in range(n_robots)]
        self.dists_ = np.zeros([n_robots,])

        self.bag_path_ = bag_path
        self.bag_ = rosbag.Bag(bag_path, 'r') 

    def analyze(self):
        for topic, msg, t in tqdm(self.bag_.read_messages(topics = self.topic_list_)):
            robot_name = topic.split('/')[1]
            if "pose_est" in topic:
                loc = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
                self.pose_cb(loc, self.robot_list_[robot_name])
            elif "is_auto" in topic:
                robot_id = self.robot_list_[robot_name]
                self.is_auto_[robot_id] = msg.data
                if not msg.data:
                    self.last_poses_[robot_id] = None

        print("====")
        print(f"Dataset {self.bag_path_}")
        for robot_name in self.robot_list_:
            print(f"{robot_name} travelled {self.dists_[self.robot_list_[robot_name]]/1000.} km")

    def pose_cb(self, loc, robot_id):
        if self.real_ and not self.is_auto_[robot_id]:
            # don't count manual
            return

        if self.last_poses_[robot_id] is not None:
            self.dists_[robot_id] += np.linalg.norm(loc - self.last_poses_[robot_id])

        self.last_poses_[robot_id] = loc

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Integrate distance robot travelled")
    parser.add_argument("bag_paths", help="Path to bag to process", nargs='+')
    parser.add_argument("-n", "--n_robots", help="Number of robots", type=int)
    parser.add_argument("-r", "--real", help="Dataset of real robots", action="store_true")
    args = parser.parse_args()

    for path in args.bag_paths:
        ga = DistAnalyzer(path, args.n_robots, args.real)
        ga.analyze()
