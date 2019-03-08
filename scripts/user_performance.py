#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist, WrenchStamped
from ipr_helpers.msg import Pose2DStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import InteractiveMarkerUpdate, Marker, MarkerArray

def callback_mobile_robot_pose(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    pass

def callback_input_force(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    pass

def callback_disturbance_force(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    pass

def callback_robot_command(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    pass

def callback_checkpoint(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    pass

def callback_path_update(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    pass

def callback_odometry(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    pass

def callback_checkpoint_info(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    pass

def callback_legs_people(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    pass

def user_performance():
    rospy.init_node('robotrainer_user_perfomance')

    rospy.Subscriber("mobile_robot_pose", Pose2DStamped, callback_mobile_robot_pose)
    rospy.Subscriber("input_force", WrenchStamped, callback_input_force)
    rospy.Subscriber("disturbance_force", WrenchStamped, callback_disturbance_force)
    rospy.Subscriber("robot_command", Twist, callback_robot_command)
    #rospy.Subscriber("checkpoint", Marker, callback_checkpoint)
    #rospy.Subscriber("path_update", InteractiveMarkerUpdate, callback_path_update) # Why?
    rospy.Subscriber("odometry", Odometry, callback_odometry)
    rospy.Subscriber("checkpoint_info", String, callback_checkpoint_info)
    rospy.Subscriber("legs_people", MarkerArray, callback_legs_people)

    #Create your own structure and fill with data from the callbacks
    # Open questions:
    ## How should we synchronize data?
    ## How often should we call your algorithm?
    ## How should look your output structure?

    rospy.spin()

if __name__ == '__main__':
    user_performance()
