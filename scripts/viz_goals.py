#!/usr/bin/env python3

import threading
from functools import partial
import numpy as np
import rospy
from spomp.msg import ClaimedGoal, ClaimedGoalArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class VizGoals:
    def __init__(self):
        self.robot_list_ = rospy.get_param("~robot_list").split(',')
        self.min_goal_dist_m_ = rospy.get_param("~min_goal_dist_m", default=5)
        self.goal_list_ = np.zeros((0, 2))
        self.goal_status_ = []
        self.lock_ = threading.Lock()

        self.goal_viz_pub_ = rospy.Publisher("~goal_viz", Marker, queue_size=1)

        self.claimed_goal_subs_ = []
        self.goal_subs_ = []
        for robot in self.robot_list_:
            claimed_goal_cb = partial(self.claimed_goal_cb, robot=robot)
            self.claimed_goal_subs_.append(rospy.Subscriber(
                f"/{robot}/goal_manager/claimed_goals", ClaimedGoalArray, claimed_goal_cb))

            goal_cb = partial(self.goal_cb, robot=robot)
            self.goal_subs_.append(rospy.Subscriber(
                f"/{robot}/goal_manager/goal_viz", Marker, goal_cb))

    def goal_cb(self, goal_msg, robot):
        self.lock_.acquire()
        for goal in goal_msg.points:
            goal_np = np.array([goal.x, goal.y])
            if self.goal_list_.shape[0] > 0:
                dists = np.linalg.norm(self.goal_list_ - goal_np, axis=1)
                if np.min(dists) < self.min_goal_dist_m_:
                    # goal already exists
                    continue

            # add goal
            self.goal_list_ = np.vstack([self.goal_list_, goal_np])
            self.goal_status_.append(-1)

        self.viz()
        self.lock_.release()

    def claimed_goal_cb(self, goal_msg, robot):
        self.lock_.acquire()
        for goal in goal_msg.goals:
            goal_np = np.array([goal.position.x, goal.position.y])
            if self.goal_list_.shape[0] > 0:
                dists = np.linalg.norm(self.goal_list_ - goal_np, axis=1)
                min_ind = np.argmin(dists)
                min_dist = dists[min_ind]
                if min_dist < self.min_goal_dist_m_:
                    if goal.status > self.goal_status_[min_ind]:
                        self.goal_status_[min_ind] = goal.status
                    continue
                
            # add goal
            self.goal_list_ = np.vstack([self.goal_list_, goal_np])
            self.goal_status_.append(goal.status)

        self.viz()
        self.lock_.release()

    def viz(self):
        marker_msg = Marker()
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.header.frame_id = "map"
        marker_msg.ns = "robot_goals"
        marker_msg.id = 0
        marker_msg.type = Marker.SPHERE_LIST
        marker_msg.action = Marker.ADD
        marker_msg.pose.position.z = 1
        marker_msg.pose.orientation.w = 1
        marker_msg.scale.x = 5
        marker_msg.scale.y = 5
        marker_msg.scale.z = 5
        marker_msg.color.a = 1

        for goal_pos, goal_status in zip(self.goal_list_, self.goal_status_):
            pt_msg = Point()
            color_msg = ColorRGBA()
            color_msg.a = 1
            pt_msg.x = goal_pos[0]
            pt_msg.y = goal_pos[1]
            if goal_status == ClaimedGoal.IN_PROGRESS:
                color_msg.r = 1
                color_msg.g = 1
                color_msg.b = 0
            elif goal_status == ClaimedGoal.VISITED:
                color_msg.r = 0
                color_msg.g = 1
                color_msg.b = 0
            else:
                color_msg.r = 0.8
                color_msg.g = 0.8
                color_msg.b = 0.8

            marker_msg.points.append(pt_msg)
            marker_msg.colors.append(color_msg)

        self.goal_viz_pub_.publish(marker_msg)        

if __name__ == '__main__':
    rospy.init_node('viz_goals')
    vg = VizGoals()
    rospy.spin()
