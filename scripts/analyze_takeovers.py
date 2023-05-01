#!/usr/bin/env python3

from functools import partial
import rospy
from std_msgs.msg import Bool
from rosgraph_msgs.msg import Clock

class AnalyzeTakeovers:
    def __init__(self):
        self.robot_list_ = rospy.get_param("~robot_list").split(',')

        self.robot_is_auto_ = {r:False for r in self.robot_list_}
        self.start_times_ = {r:0 for r in self.robot_list_}
        self.takeover_times_ = {r:[] for r in self.robot_list_}
        self.takeover_subs_ = []
        for robot in self.robot_list_:
            takeover_cb = partial(self.takeover_cb, robot=robot)
            self.takeover_subs_.append(rospy.Subscriber(
                f"/{robot}/jackal_teleop/is_auto", Bool, takeover_cb))

        start_t = rospy.wait_for_message("/clock", Clock)
        self.dataset_start_t_ = start_t.clock.to_sec()
        rospy.loginfo("Dataset started")

    def takeover_cb(self, is_auto_msg, robot):
        if is_auto_msg == self.robot_is_auto_[robot]:
            # nothing changed, ignore
            return
        self.robot_is_auto_[robot] = is_auto_msg

        if is_auto_msg.data and self.start_times_[robot] == 0:
            # first auto 
            self.start_times_[robot] = rospy.Time.now()
            rospy.loginfo(f"Robot {robot} started mission")

        if not is_auto_msg.data:
            # manual
            self.takeover_times_[robot].append([rospy.Time.now(), None])
        elif len(self.takeover_times_[robot]) > 0:
            # auto
            duration = rospy.Time.now() - self.takeover_times_[robot][-1][0]
            self.takeover_times_[robot][-1][1] = duration
            rospy.loginfo(f"Robot {robot} manual takeover for {duration.to_sec()} sec")

    def finish(self):
        stop_t = rospy.Time.now()
        for robot in self.robot_list_:
            rospy.loginfo(f"Robot {robot} summary:")
            if self.start_times_[robot] == 0:
                rospy.loginfo(f"Never started")
                continue

            start_t = self.start_times_[robot]
            total_takeover_t = 0
            for takeover in self.takeover_times_[robot]:
                if takeover[1] is None:
                    continue
                rospy.loginfo(f"Takeover at {takeover[0].to_sec()-self.dataset_start_t_} of duration {takeover[1].to_sec()} sec")
                total_takeover_t += takeover[1].to_sec()
            total_t = (stop_t - start_t).to_sec()
            rospy.loginfo(f"Manual {100*total_takeover_t/total_t}% of the time")

if __name__ == '__main__':
    rospy.init_node('analyze_takeovers')
    at = AnalyzeTakeovers()
    rospy.spin()
    at.finish()
