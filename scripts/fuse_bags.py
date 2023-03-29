#!/usr/bin/env python3

import os
from pathlib import Path
import argparse
from tqdm import tqdm
import rosbag

class FusedBag:
    def __init__(self, path, exp, robot_name, topic_blacklist):
        self.root_p_ = Path(path)
        self.exp_ = exp
        self.topic_blacklist_ = topic_blacklist.split(',')

        try:
            os.mkdir(self.root_p_ / "combined")
        except FileExistsError:
            # Directory already exists, this is good!
            pass
        
        fused_path = self.root_p_ / "combined" / (exp + '_combined.bag')
        if fused_path.is_file():
            if input(f"File {fused_path} already exists.  Delete? (y/n): ") == "y":
                os.remove(fused_path)
            else:
                print("Quitting")
                exit()
        print(f"Creating new bag at {fused_path}")
        self.fused_bag_ = rosbag.Bag(fused_path, 'w')

        for robot in robot_name.split(','):
            self.add_robot(robot)

        self.fused_bag_.close()
    
    def add_robot(self, robot):
        robot_p = self.root_p_ / robot
        robot_bags = sorted(robot_p.glob("**/" + self.exp_ + '*.bag'))
        for bag in robot_bags:
            self.add_bag(robot, bag)

    def prepend_robot_to_frame(self, robot, frame):
        if frame == "map":
            return frame
        return f"{robot}/{frame}"

    def add_bag(self, robot, bag_path):
        print(f"Parsing {bag_path}")
        for topic, msg, t in tqdm(rosbag.Bag(bag_path).read_messages()):
            if topic in self.topic_blacklist_:
                continue
            
            if topic == '/tf' or topic == '/tf_static':
                for trans in msg.transforms:
                    trans.header.frame_id = self.prepend_robot_to_frame(robot, trans.header.frame_id)
                    trans.child_frame_id = self.prepend_robot_to_frame(robot, trans.child_frame_id)
                self.fused_bag_.write(topic, msg, t)
            else:
                if hasattr(msg, "header") and hasattr(msg.header, "frame_id"):
                    msg.header.frame_id = self.prepend_robot_to_frame(robot, msg.header.frame_id)
                self.fused_bag_.write(f"/{robot}{topic}", msg, t)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Fuse multiple bags from different robots")
    parser.add_argument('root')
    parser.add_argument('exp')
    parser.add_argument('--robot_list', default='io,callisto,europa,titan', type=str)
    parser.add_argument('--topic_blacklist', default='/os_node/metadata,/os_node/imu_packets,/os_node/lidar_packets', type=str)
    args = parser.parse_args()

    db = FusedBag(args.root, args.exp, args.robot_list, args.topic_blacklist)
