#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg

def calc_force(fx, fy, fz):
    ret = fx**2.0 + fy**2.0 + fz**2.0
    ret = ret**0.5
    return ret

def calc_deviation(front, left, right, c=0.1):
    front = float(front)
    left = float(left)
    right = float(right)
    
    dist = min([front, left, right])
    
    return  1/(dist+c)

def callback_mobile_robot_pose(data):
    #rospy.loginfo(data)
    global pose_x
    global pose_y
    global pose_theta
    
    
    
    pose_x = data.pose.x
    pose_y = data.pose.y
    pose_theta = data.pose.theta
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", pose_x)
    pass

def callback_input_force(data):

    #rospy.loginfo('here1')
    #rospy.loginfo(data)
    #rospy.loginfo('end1')
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
    #rospy.loginfo(input_force_x)
    
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", input_force_x)
    pass

def callback_disturbance_force(data):
    #rospy.loginfo('here2')
    #rospy.loginfo(data)
    #rospy.loginfo('end2')
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


def callback_deviation(data):
    global dist_front
    global dist_left
    global dist_right
    
    dist_front = data.front
    dist_left = data.left
    dist_right = data.right


if __name__ == '__main__':
    rospy.init_node('robotrainer_classic_user_perfomance')
    listener = tf.TransformListener()
    
    rospy.Subscriber("mobile_robot_pose", Pose2DStamped, callback_mobile_robot_pose)
    rospy.Subscriber("deviation", RobotrainerUserDeviation, callback_deviation)
    rospy.Subscriber("input_force", WrenchStamped, callback_input_force)
    rospy.Subscriber("disturbance_force", WrenchStamped, callback_disturbance_force)


    rate = rospy.Rate(10)
    calc_rate = 1
    pre_time = time.time()
    
    
