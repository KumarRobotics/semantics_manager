#!/usr/bin/env python3

from functools import partial
import numpy as np
import utm
from pathlib import Path
import yaml
import rospy
import rospkg
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseStamped, Point
from std_msgs.msg import Int32, Time, ColorRGBA
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker, MarkerArray

class VizComms:
    def __init__(self):
        self.robot_list_ = rospy.get_param("~robot_list").split(',')
        self.aerial_robot_name_ = rospy.get_param("~aerial_map_ns")
        self.rssi_thresh_ = rospy.get_param("~rssi_thresh", 20)
        self.robot_list_.append(self.aerial_robot_name_)

        world_config_path = rospy.get_param("~world_config_path",
                Path(rospkg.RosPack().get_path('semantics_manager')) / "config/config.yaml")
        world_config = yaml.load(open(world_config_path, 'r'), Loader=yaml.CLoader)
        map_path = Path(world_config_path).parent.absolute() / world_config["map"]
        map_config = yaml.load(open(map_path, 'r'), Loader=yaml.CLoader)
        self.gps_origin_utm_ = np.array(utm.from_latlon(map_config["gps_origin_lat"],
            map_config["gps_origin_long"])[:2])

        self.rssi_subs_ = []
        self.pose_subs_ = []
        self.sync_complete_subs_ = []
        self.rssis_ = {}
        self.robot_positions_ = {}
        for robot1 in self.robot_list_:
            self.robot_positions_[robot1] = np.array([0., 0, 0])
            self.rssis_[robot1] = {}
            if robot1 == self.aerial_robot_name_:
                self.gps_pub_ = rospy.Publisher(f"/{robot1}/top_down_render/pose_est",
                        PoseWithCovarianceStamped, queue_size=10)
                self.gps_sub_ = rospy.Subscriber(
                    f"/{robot1}/mavros/global_position/raw/fix", NavSatFix, self.gps_cb)

            pose_cb = partial(self.pose_cb, robot=robot1)
            self.pose_subs_.append(rospy.Subscriber(
                f"/{robot1}/top_down_render/pose_est", PoseWithCovarianceStamped, pose_cb))
            for robot2 in self.robot_list_:
                if robot1 != robot2:
                    self.rssis_[robot1][robot2] = {'rssi': 0, 'stamp': 0, 'last_sc': 0}
                    rssi_cb = partial(self.rssi_cb, robot1=robot1, robot2=robot2)
                    self.rssi_subs_.append(rospy.Subscriber(f"/{robot1}/ddb/rajant/rssi/{robot2}", Int32, rssi_cb))

                    sync_complete_cb = partial(self.sync_complete_cb, server_r=robot1, client_r=robot2)
                    self.sync_complete_subs_.append(rospy.Subscriber(
                        f"/{robot1}/ddb/server_sync_complete/{robot2}", Time, sync_complete_cb))

        self.comm_viz_pub_ = rospy.Publisher("~comm_viz", MarkerArray, queue_size=1)
        self.viz_timer_ = rospy.Timer(rospy.Duration(1), self.viz_cb, oneshot=False)

    def gps_cb(self, gps_msg):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = gps_msg.header.stamp
        pose_msg.header.frame_id = "map"

        # parse GPS
        utm_coords = np.array(utm.from_latlon(gps_msg.latitude, gps_msg.longitude)[:2])
        utm_coords -= self.gps_origin_utm_
        pose_msg.pose.pose.position.x = utm_coords[0]
        pose_msg.pose.pose.position.y = utm_coords[1]
        pose_msg.pose.pose.position.z = 10
        pose_msg.pose.pose.orientation.w = 1

        self.gps_pub_.publish(pose_msg)

    def pose_cb(self, pose_msg, robot):
        pose_only_msg = pose_msg.pose.pose
        self.robot_positions_[robot] = np.array([pose_only_msg.position.x,
                                                 pose_only_msg.position.y,
                                                 pose_only_msg.position.z])

    def rssi_cb(self, rssi_msg, robot1, robot2):
        self.rssis_[robot1][robot2]['rssi'] = rssi_msg.data
        self.rssis_[robot1][robot2]['stamp'] = rospy.get_time()

    def sync_complete_cb(self, msg, server_r, client_r):
        self.rssis_[server_r][client_r]['last_sc'] = rospy.get_time()

    def viz_cb(self, timer):
        comm_edges = []
        sc_edges = []
        sc_colors = []
        for robot1 in self.robot_list_:
            for robot2 in self.robot_list_:
                if robot1 != robot2:
                    rssi = self.rssis_[robot1][robot2]
                    if rospy.get_time() - rssi['stamp'] > 10:
                        rssi['rssi'] = 0

                    r1pos = self.robot_positions_[robot1]
                    r2pos = self.robot_positions_[robot2]
                    if rospy.get_time() - rssi['last_sc'] < 10:
                        sc_edges.append(Point(r1pos[0], r1pos[1], r1pos[2]))
                        sc_edges.append(Point(r2pos[0], r2pos[1], r2pos[2]))
                        sc_colors.append(ColorRGBA(0, 0, 1, 0.5))
                        sc_colors.append(ColorRGBA(0, 1, 1, 0.5))
                    if rssi['rssi'] > self.rssi_thresh_:
                        comm_edges.append(Point(r1pos[0], r1pos[1], r1pos[2]))
                        comm_edges.append(Point(r2pos[0], r2pos[1], r2pos[2]))

        comm_viz_rssi_msg = Marker()
        comm_viz_rssi_msg.header.frame_id = "map"
        comm_viz_rssi_msg.ns = "comm_viz_rssi"
        comm_viz_rssi_msg.type = Marker.LINE_LIST
        comm_viz_rssi_msg.action = Marker.ADD
        comm_viz_rssi_msg.pose.orientation.w = 1
        comm_viz_rssi_msg.scale.x = 0.5
        comm_viz_rssi_msg.color.a = 1
        comm_viz_rssi_msg.color.g = 1
        comm_viz_rssi_msg.points = comm_edges

        comm_viz_sc_msg = Marker()
        comm_viz_sc_msg.header.frame_id = "map"
        comm_viz_sc_msg.ns = "comm_viz_sc"
        comm_viz_sc_msg.type = Marker.LINE_LIST
        comm_viz_sc_msg.action = Marker.ADD
        comm_viz_sc_msg.pose.orientation.w = 1
        comm_viz_sc_msg.scale.x = 1.5
        comm_viz_sc_msg.color.a = 1
        comm_viz_sc_msg.color.g = 1
        comm_viz_sc_msg.points = sc_edges
        comm_viz_sc_msg.colors = sc_colors

        comm_viz_msg = MarkerArray()
        comm_viz_msg.markers.append(comm_viz_rssi_msg)
        comm_viz_msg.markers.append(comm_viz_sc_msg)

        self.comm_viz_pub_.publish(comm_viz_msg)

if __name__ == '__main__':
    rospy.init_node('viz_comms')
    vc = VizComms()
    rospy.spin()
