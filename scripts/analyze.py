#!/usr/bin/env python3

import argparse
import re
from pathlib import Path

import numpy as np
import rosbag
from spomp.msg import ClaimedGoal
from tqdm import tqdm

# This is heavily based on viz_goals, but with a different objective in mind


class GoalAnalyzer:
    def __init__(self, bag_path, n_robots, real=False, quad_name="quadrotor1"):
        # params
        self.real_ = real
        self.min_goal_dist_m_ = 17.85  # tuned to unity urban scene
        self.robot_list_ = {}
        self.topic_list_ = []
        self.quad_name_ = quad_name
        if real:
            self.quad_name_ = "titan"
            self.robot_list_["io"] = 0
            self.robot_list_["europa"] = 1
            self.robot_list_["callisto"] = 2
        else:
            for i in range(n_robots):
                robot_name = f"husky{i+1}"
                self.robot_list_[robot_name] = i

            self.topic_list_.append("/fake_db/latency")

        self.topic_list_.append(f"/{self.quad_name_}/asoom/map")
        self.topic_list_.append(f"/{self.quad_name_}/asoom/map/compressed")
        for robot_name in self.robot_list_:
            self.topic_list_.append(f"/{robot_name}/goal_manager/goal_viz")
            self.topic_list_.append(f"/{robot_name}/goal_manager/claimed_goals")

        # vars
        self.goal_list_ = np.zeros((0, 2))
        self.start_t_ = None

        # outputs
        self.goal_discovery_t_ = np.zeros((len(self.robot_list_), 0))
        self.goal_claimed_t_ = np.zeros((len(self.robot_list_), 0))
        self.goal_unclaimed_t_ = np.zeros((len(self.robot_list_), 0))
        self.goal_visited_t_ = np.zeros((len(self.robot_list_), 0))
        self.total_latency_ = []
        self.total_wait_time_ = []
        self.total_msgs_ = []

        self.goal_claimed_sup_ = {}

        # for distances
        for robot_name in self.robot_list_:
            self.topic_list_.append(f"/{robot_name}/top_down_render/pose_est")
            self.topic_list_.append(f"/{robot_name}/jackal_teleop/is_auto")

        self.last_poses_ = [None for r in range(n_robots)]
        self.is_auto_ = [False for r in range(n_robots)]
        self.dists_ = np.zeros(
            [
                n_robots,
            ]
        )

        # distance travelled until goal is reached
        self.goal_dists_ = []

        self.bag_path_ = Path(bag_path)
        print("Loading bag...")
        self.bag_ = rosbag.Bag(bag_path, "r")
        print("Bag loaded")

    def analyze(self):
        end_t = 0
        for topic, msg, t in tqdm(self.bag_.read_messages(topics=self.topic_list_)):
            robot_name = topic.split("/")[1]
            # for distances
            if "pose_est" in topic:
                loc = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
                self.pose_cb(loc, self.robot_list_[robot_name])

            elif "is_auto" in topic:
                robot_id = self.robot_list_[robot_name]
                self.is_auto_[robot_id] = msg.data
                if not msg.data:
                    self.last_poses_[robot_id] = None

            elif (
                topic == f"/{self.quad_name_}/asoom/map"
                or topic == f"/{self.quad_name_}/asoom/map/compressed"
            ):
                # only use stamp, so compressed is fine
                if self.start_t_ is None and msg.info.header.stamp.to_sec() > 0:
                    self.start_t_ = msg.info.header.stamp.to_sec()
                    print(f"START TIME: {self.start_t_}")

            elif "goal_manager/goal_viz" in topic:
                # only use stamp, so compressed is fine
                if self.start_t_ is None and msg.header.stamp.to_sec() > 0:
                    self.start_t_ = msg.header.stamp.to_sec()
                    print(f"START TIME: {self.start_t_}")

                end_t = max(msg.header.stamp.to_sec(), end_t)
                robot_name = topic.split("/")[1]
                self.goal_cb(msg, self.robot_list_[robot_name])

            elif "goal_manager/claimed_goals" in topic:
                robot_name = topic.split("/")[1]
                self.claimed_goal_cb(msg, self.robot_list_[robot_name])

            elif "fake_db/latency" in topic:
                latency_re = "latency: (\d+\.\d+)s"
                latency = re.findall(latency_re, str(msg))

                if len(latency) == 1:
                    latency = np.float32(latency[0])
                    self.total_latency_.append(latency)

                wait_time_re = "(\d+) msgs waited (\d+\.\d+)"
                wait_time = re.findall(wait_time_re, str(msg))

                if len(wait_time) == 1:
                    n_msgs, wait_time = wait_time[0]
                    n_msgs = np.int32(n_msgs)
                    wait_time = np.float32(wait_time)
                    self.total_wait_time_.append(wait_time)
                    self.total_msgs_.append(n_msgs)

        filename = self.bag_path_.with_name(self.bag_path_.stem + "_analysis")
        np.savez(
            filename,
            times=np.stack(
                [
                    self.goal_discovery_t_,
                    self.goal_claimed_t_,
                    self.goal_unclaimed_t_,
                    self.goal_visited_t_,
                ]
            ),
            supp=self.goal_claimed_sup_,
            end_t=end_t - self.start_t_,
            goal_dists=np.array(self.goal_dists_),
            robot_dists=self.dists_,
            total_latency_s=np.array(self.total_latency_),
            total_wait_time_s=np.array(self.total_wait_time_),
            total_msgs=np.array(self.total_msgs_),
            goal_list=self.goal_list_,
        )

        print("====")
        print(f"Dataset {self.bag_path_}")
        for robot_name in self.robot_list_:
            print(
                f"{robot_name} travelled {self.dists_[self.robot_list_[robot_name]]/1000.} km"
            )

    def goal_cb(self, goal_msg, robot_id):
        for goal in goal_msg.points:
            goal_np = np.array([goal.x, goal.y])
            if self.goal_list_.shape[0] > 0:
                dists = np.linalg.norm(self.goal_list_ - goal_np, axis=1)
                min_ind = np.argmin(dists)
                min_dist = dists[min_ind]

                if min_dist < self.min_goal_dist_m_:
                    # goal already exists
                    self.update_goal_stamps(
                        -1,
                        min_ind,
                        robot_id,
                        goal_msg.header.stamp.to_sec() - self.start_t_,
                    )
                    continue

            self.add_goal(
                goal_np, -1, goal_msg.header.stamp.to_sec() - self.start_t_, robot_id
            )

    def claimed_goal_cb(self, goal_msg, robot_id):
        goals_cur_claimed_tmp = set()

        for goal in goal_msg.goals:
            goal_np = np.array([goal.position.x, goal.position.y])
            if self.goal_list_.shape[0] > 0:
                dists = np.linalg.norm(self.goal_list_ - goal_np, axis=1)
                min_ind = np.argmin(dists)
                min_dist = dists[min_ind]

                if min_dist < self.min_goal_dist_m_:
                    goals_cur_claimed_tmp.add(min_ind)
                    self.update_goal_stamps(
                        goal.status,
                        min_ind,
                        robot_id,
                        goal_msg.header.stamp.to_sec() - self.start_t_,
                    )
                    continue

            self.add_goal(
                goal_np,
                goal.status,
                goal_msg.header.stamp.to_sec() - self.start_t_,
                robot_id,
            )
            goals_cur_claimed_tmp.add(len(self.goal_list_) - 1)

        # Find goals we have previously claimed but are not any more
        goals_already_claimed = np.nonzero(self.goal_claimed_t_[robot_id])
        goals_unclaimed = set(goals_already_claimed[0]).difference(
            goals_cur_claimed_tmp
        )
        for g_id in goals_unclaimed:
            if self.goal_unclaimed_t_[robot_id][g_id] == 0:
                self.goal_unclaimed_t_[robot_id][g_id] = (
                    goal_msg.header.stamp.to_sec() - self.start_t_
                )
                print(f"Robot {robot_id} unclaimed goal {g_id}")
            elif (
                robot_id in self.goal_claimed_sup_
                and g_id in self.goal_claimed_sup_[robot_id]
                and self.goal_claimed_sup_[robot_id][g_id][-1][0] == "claimed"
            ):
                self.goal_claimed_sup_[robot_id][g_id].append(
                    ("unclaimed", goal_msg.header.stamp.to_sec() - self.start_t_)
                )

        for g_id, g_t in enumerate(self.goal_unclaimed_t_[robot_id]):
            if g_t > 0 and g_id in goals_cur_claimed_tmp:
                # goal has been claimed again
                if robot_id not in self.goal_claimed_sup_:
                    self.goal_claimed_sup_[robot_id] = {}
                if g_id not in self.goal_claimed_sup_[robot_id]:
                    self.goal_claimed_sup_[robot_id][g_id] = [
                        ("claimed", goal_msg.header.stamp.to_sec() - self.start_t_)
                    ]
                elif self.goal_claimed_sup_[robot_id][g_id][-1][0] == "unclaimed":
                    self.goal_claimed_sup_[robot_id][g_id].append(
                        ("claimed", goal_msg.header.stamp.to_sec() - self.start_t_)
                    )

    def add_goal(self, goal_loc, goal_status, t, robot_id):
        self.goal_discovery_t_ = np.hstack(
            (self.goal_discovery_t_, np.zeros((len(self.robot_list_), 1)))
        )
        self.goal_claimed_t_ = np.hstack(
            (self.goal_claimed_t_, np.zeros((len(self.robot_list_), 1)))
        )
        self.goal_unclaimed_t_ = np.hstack(
            (self.goal_unclaimed_t_, np.zeros((len(self.robot_list_), 1)))
        )
        self.goal_visited_t_ = np.hstack(
            (self.goal_visited_t_, np.zeros((len(self.robot_list_), 1)))
        )

        self.goal_list_ = np.vstack([self.goal_list_, goal_loc])

        self.update_goal_stamps(goal_status, len(self.goal_list_) - 1, robot_id, t)

        self.goal_dists_.append(self.dists_[robot_id])

    def update_goal_stamps(self, goal_status, goal_id, robot_id, time_sec):
        if self.goal_discovery_t_[robot_id][goal_id] == 0:
            self.goal_discovery_t_[robot_id][goal_id] = time_sec
            print(f"Robot {robot_id} discovered goal {goal_id}")
        if (
            goal_status == ClaimedGoal.IN_PROGRESS
            and self.goal_claimed_t_[robot_id][goal_id] == 0
        ):
            self.goal_claimed_t_[robot_id][goal_id] = time_sec
            print(f"Robot {robot_id} claimed goal {goal_id}")
        if (
            goal_status == ClaimedGoal.VISITED
            and self.goal_visited_t_[robot_id][goal_id] == 0
        ):
            self.goal_visited_t_[robot_id][goal_id] = time_sec
            print(f"Robot {robot_id} visited goal {goal_id}")

    def pose_cb(self, loc, robot_id):
        if self.real_ and not self.is_auto_[robot_id]:
            # don't count manual
            return

        if self.last_poses_[robot_id] is not None:
            self.dists_[robot_id] += np.linalg.norm(loc - self.last_poses_[robot_id])

        self.last_poses_[robot_id] = loc


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate statistics for reaching goals"
    )
    parser.add_argument("bag_paths", help="Path to bag to process", nargs="+")
    parser.add_argument("-n", "--n_robots", help="Number of robots", type=int)
    parser.add_argument(
        "-r", "--real", help="Dataset of real robots", action="store_true"
    )
    parser.add_argument(
        "-q", "--quad_name", help="quadrotor name(s)", default="quadrotor1"
    )
    args = parser.parse_args()

    for path in args.bag_paths:
        ga = GoalAnalyzer(path, args.n_robots, args.real, quad_name=args.quad_name)
        ga.analyze()
