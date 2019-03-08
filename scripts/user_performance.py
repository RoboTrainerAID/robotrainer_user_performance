#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist, WrenchStamped
from ipr_helpers.msg import Pose2DStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import InteractiveMarkerUpdate, Marker, MarkerArray
import numpy as np
import pandas as pd


def init_var():
    global pose_x
    global pose_y
    global pose_theta
    global input_force_x
    global input_force_y
    global input_force_z
    global input_torque_x
    global input_torque_y
    global input_torque_z
    global dis_force_x
    global dis_force_y
    global dis_force_z
    global dis_torque_x
    global dis_torque_y
    global dis_torque_z
    global vel_linear_x
    global vel_linear_y
    global vel_linear_z
    global vel_angular_x
    global vel_angular_y
    global vel_angular_z
    global wheel_position_x
    global wheel_position_y
    global dist_front
    global dist_left
    global dist_right
    global leg_position_x
    global leg_position_y
    
    pose_x = np.nan
    pose_y = np.nan
    pose_theta = np.nan
    input_force_x = np.nan
    input_force_y = np.nan
    input_force_z = np.nan
    input_torque_x = np.nan
    input_torque_y = np.nan
    input_torque_z = np.nan
    dis_force_x = np.nan
    dis_force_y = np.nan
    dis_force_z = np.nan
    dis_torque_x = np.nan
    dis_torque_y = np.nan
    dis_torque_z = np.nan
    vel_linear_x = np.nan
    vel_linear_y = np.nan
    vel_linear_z = np.nan
    vel_angular_x = np.nan
    vel_angular_y = np.nan
    vel_angular_z = np.nan
    wheel_position_x = np.nan
    wheel_position_y = np.nan
    dist_front = np.nan
    dist_left = np.nan
    dist_right = np.nan
    leg_position_x = np.nan
    leg_position_y = np.nan
    
def get_record(ls, cols):
    return pd.DataFrame(np.array(ls).reshape(1, -1), columns=cols)
    pass

def update_history(df, history, length=100):
    history = pd.concat([history, df])
    if len(history)>length:
        return history.loc[:length]
    else:
        return history

def callback_mobile_robot_pose(data):
    
    global pose_x
    global pose_y
    global pose_theta
    
    pose_x = data.pose.x
    pose_y = data.pose.y
    pose_theta = data.pose.theta
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", pose_x)
    pass

def callback_input_force(data):
    
    global input_force_x
    global input_force_y
    global input_force_z
    global input_torque_x
    global input_torque_y
    global input_torque_z
    
    input_force_x = data.wrench.force.x
    input_force_y = data.wrench.force.y
    input_force_z = data.wrench.force.z
    input_torque_x = data.wrench.torque.x
    input_torque_y = data.wrench.torque.y
    input_torque_z = data.wrench.torque.z
    
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", input_force_x)
    pass

def callback_disturbance_force(data):
    
    global dis_force_x
    global dis_force_y
    global dis_force_z
    global dis_torque_x
    global dis_torque_y
    global dis_torque_z
    
    dis_force_x = data.wrench.force.x
    dis_force_y = data.wrench.force.y
    dis_force_z = data.wrench.force.z
    dis_torque_x = data.wrench.torque.x
    dis_torque_y = data.wrench.torque.y
    dis_torque_z = data.wrench.torque.z
    
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", dis_force_x)
    pass

def callback_robot_command(data):
    
    global vel_linear_x
    global vel_linear_y
    global vel_linear_z
    global vel_angular_x
    global vel_angular_y
    global vel_angular_z
    
    vel_linear_x = data.linear.x
    vel_linear_y = data.linear.y
    vel_linear_z = data.linear.z
    vel_angular_x = data.angular.x
    vel_angular_y = data.angular.y
    vel_angular_z = data.angular.z
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", vel_linear_x)
    pass

def callback_checkpoint(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    pass

def callback_path_update(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    pass

def callback_odometry(data):
    
    global wheel_position_x
    global wheel_position_y
    
    wheel_position_x = data.pose.pose.position.x
    wheel_position_y = data.pose.pose.position.y
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", wheel_position_x)
    pass

def callback_checkpoint_info(data):
    
    global dist_front
    global dist_left
    global dist_right
    
    dist_front = data.data[data.data.find('front')+6:data.data.find('\n')]
    dist_left = data.data[data.data.find('left')+5:data.data.find('\n',data.data.find('left'))]
    dist_right = data.data[data.data.find('right')+6:]
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", dist_front)
    pass

def callback_legs_people(data):
    
    leg_position_x = np.nan
    leg_position_y = np.nan
    
    for tmp in data.markers:
        leg_position_x = tmp.pose.position.x
        leg_position_y = tmp.pose.position.y
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", leg_position_x)
    pass

def user_performance():
    # init
    cols = 'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz', 'DFx', 'DFy', 'DFz', 'DMx', 'DMy', 'DMz' , 'Lx', 'Ly', 'Lz', 'Ax', 'Ay', 'Az' , 'front', 'left', 'right', 'pose.x', 'pose.y', 'pose.theta', 'leg_pose_x', 'leg_pose_y'
    record_history = pd.DataFrame()
    
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
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        curr = get_record([input_force_x, input_force_y, input_force_z, input_torque_x, input_torque_y, input_torque_z, dis_force_x, dis_force_y, dis_force_z, 
                           dis_torque_x, dis_torque_y, dis_torque_z, vel_linear_x, vel_linear_y, vel_linear_z, vel_angular_x, vel_angular_y, vel_angular_z,
                           dist_front, dist_left, dist_right, pose_x, pose_y, pose_theta, leg_position_x, leg_position_y], cols)
        record_history = update_history(curr, record_history, length=100)
        rospy.loginfo(curr)
        rate.sleep()
	
    rospy.spin()

if __name__ == '__main__':
    init_var()
    user_performance()
