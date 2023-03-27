#!/usr/bin/env python3

from functools import partial
import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker

class VizComms:
    def __init__(self):
        self.robot_list_ = rospy.get_param("~robot_list").split(',')
        self.aerial_robot_name_ = rospy.get_param("~aerial_map_ns")
        self.robot_list_.append(self.aerial_robot_name_)

        self.rssi_subs_ = []
        self.pose_subs_ = []
        self.rssis_ = {}
        self.robot_positions_ = {}
        for robot1 in self.robot_list_:
            self.robot_positions_[robot1] = np.array([0., 0, 0])
            self.rssis_[robot1] = {}
            pose_cb = partial(self.pose_cb, robot=robot1)
            if robot1 == self.aerial_robot_name_:
                self.pose_subs_.append(rospy.Subscriber(
                    f"/{robot1}/asoom/recent_key_pose", PoseStamped, pose_cb))
            else:
                self.pose_subs_.append(rospy.Subscriber(
                    f"/{robot1}/top_down_render/pose_est", PoseWithCovarianceStamped, pose_cb))
            for robot2 in self.robot_list_:
                if robot1 != robot2:
                    self.rssis_[robot1][robot2] = {'rssi': 0, 'stamp': 0}
                    rssi_cb = partial(self.rssi_cb, robot1=robot1, robot2=robot2)
                    self.rssi_subs_.append(rospy.Subscriber(f"/{robot1}/rajant/rssi/{robot2}", Int32, rssi_cb))

        self.comm_viz_pub_ = rospy.Publisher("~comm_viz", Marker, queue_size=1)
        self.viz_timer_ = rospy.Timer(rospy.Duration(5), self.viz_cb, oneshot=False)

    def pose_cb(self, pose_msg, robot):
        pose_only_msg = pose_msg.pose
        if isinstance(pose_msg, PoseWithCovarianceStamped):
            pose_only_msg = pose_msg.pose.pose
        self.robot_positions_[robot] = np.array([pose_only_msg.position.x,
                                                 pose_only_msg.position.y,
                                                 pose_only_msg.position.z])

    def rssi_cb(self, rssi_msg, robot1, robot2):
        self.rssis_[robot1][robot2] = {'rssi': rssi_msg.data, 'stamp': rospy.get_time()}

    def viz_cb(self, timer):
        comm_edges = []
        for robot1 in self.robot_list_:
            for robot2 in self.robot_list_:
                if robot1 != robot2:
                    rssi1 = self.rssis_[robot1][robot2]
                    rssi2 = self.rssis_[robot2][robot1]
                    if rospy.get_time() - rssi1['stamp'] > 10:
                        rssi1['rssi'] = 0
                    if rospy.get_time() - rssi2['stamp'] > 10:
                        rssi2['rssi'] = 0

                    if rssi1['rssi'] > 20 or rssi2['rssi'] > 20:
                        r1pos = self.robot_positions_[robot1]
                        r2pos = self.robot_positions_[robot2]
                        comm_edges.append(Point(r1pos[0], r1pos[1], r1pos[2]))
                        comm_edges.append(Point(r2pos[0], r2pos[1], r2pos[2]))

        comm_viz_msg = Marker()
        comm_viz_msg.header.frame_id = "map"
        comm_viz_msg.ns = "comm_viz"
        comm_viz_msg.type = Marker.LINE_LIST
        comm_viz_msg.action = Marker.ADD
        comm_viz_msg.pose.orientation.w = 1
        comm_viz_msg.scale.x = 1
        comm_viz_msg.color.a = 1
        comm_viz_msg.color.g = 1
        comm_viz_msg.points = comm_edges
        self.comm_viz_pub_.publish(comm_viz_msg)

if __name__ == '__main__':
    rospy.init_node('viz_comms')
    vc = VizComms()
    rospy.spin()
