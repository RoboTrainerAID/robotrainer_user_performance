#!/usr/bin/env python
# This class deals with collecting Rostopic



from geometry_msgs.msg import Twist, WrenchStamped, Vector3
from ipr_helpers.msg import Pose2DStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import InteractiveMarkerUpdate, Marker, MarkerArray
from robotrainer_deviation.msg import RobotrainerUserDeviation
from robotrainer_deviation.msg import PathIndex
from std_msgs.msg import Float64
import json
import roslib
import rospy
from DataFormats import RobotrainerData
import math
import tf
import geometry_msgs.msg
import time
import signal
import sys
class SubscriptionHandler():
    def __init__(self):
        self.features = RobotrainerData()
        self.listener()
    ##########################################################################
    def reset(self):
        self.features = RobotrainerData()
    def callback_mobile_robot_pose(self, data):

        self.features.mobil_robot_pose[0] = data.pose.x
        self.features.mobil_robot_pose[1] = data.pose.y

        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose.x)
        pass


    def callback_input_force(self, data):
        self.features.input_force[0] += data.wrench.force.x
        self.features.input_force[1] += data.wrench.force.y
        self.features.input_force[2] += data.wrench.force.z
        self.features.input_force_callback_number += 1

        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", features.input_force[2]) #debug
        pass


    def callback_disturbance_force(self, data):
        self.features.disturbance_force[0] += data.x
        self.features.disturbance_force[1] += data.y
        self.features.disturbance_force[2] += data.z
        self.features.disturbance_force_callback_number += 1
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", dis_force_x)
        pass


    def callback_deviation(self, data):

        self.features.deviation[0] = data.front
        self.features.deviation[1] = data.left
        self.features.deviation[2] = data.right
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.front)
        pass

    def callback_path_index(self, data):
        self.features.path_index[0] = data.front
        self.features.path_index[1] = data.left
        self.features.path_index[2] = data.right

    def callback_velocity(self, data):
        if data.x is None:
            self.features.velocity[0] += 0.0
        else:
            self.features.velocity[0] += data.x
        if data.y is None:
            self.features.velocity[1] += 0.0
        else:
            self.features.velocity[1] += data.y
        if data.z is None:
            self.features.velocity[2] += 0.0
        else:
            self.features.velocity[2] = data.z
        self.features.velocity_callback_number += 1

        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", vel_linear_x)
        pass
    def pull_data(self):
        return self.features
    def listener(self):
        rospy.Subscriber("/base/robotrainer_controllers/base/velocity_output", Vector3, self.callback_velocity)
        rospy.Subscriber("/robotrainer_deviation/robotrainer_deviation", RobotrainerUserDeviation, self.callback_deviation)
        rospy.Subscriber("/base/threshold_filtered", WrenchStamped, self.callback_input_force)
        #rospy.Subscriber("/base/virtual_forces/modalities_debug/resulting_force", Twist, self.callback_disturbance_force)
        rospy.Subscriber("/base/virtual_forces/modalities_debug/resulting_force", Vector3, self.callback_disturbance_force)
        rospy.Subscriber("/robotrainer_deviation/current_path_index", PathIndex, self.callback_path_index)
        rospy.Subscriber("/robotrainer/mobile_robot_pose", Pose2DStamped, self.callback_mobile_robot_pose)

