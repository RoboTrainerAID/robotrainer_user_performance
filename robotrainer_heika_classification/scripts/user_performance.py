#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist, WrenchStamped
from ipr_helpers.msg import Pose2DStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import InteractiveMarkerUpdate, Marker, MarkerArray
from robotrainer_deviation.msg import RobotrainerUserDeviation
import numpy as np
import pandas as pd
import os
import math
import pickle
import time
from std_msgs.msg import Float64


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
    return pd.DataFrame(np.array(ls).reshape(1, -1), columns=cols, dtype=float)
    pass

def update_history(df, history, length=100):    
    history = pd.concat([df, history]).reset_index(drop=True)
    if len(history)>=length:
        return history.loc[:length-1,]
    else:
        return history
    return df 

def update_avg_dict(avg_dict, diff, tested_feature):
    if diff not in avg_dict:
        avg_dict[diff] = [tested_feature, 1]
    else:
        avg, count = avg_dict[diff]
        avg = (avg*count+tested_feature)/(count+1)
        count = count+1
        avg_dict[diff] = [avg, count]
    return avg_dict

def is_disturbed(dis_force_x, dis_force_y, dis_force_z):
    if dis_force_x>0 or dis_force_y>0 or dis_force_z>0:
        return 1
    else:
        return 0

def is_backward(vel_x, vel_y):
    angle = np.arctan2(vel_y, vel_x)
    if angle>np.pi/2 or angle<-np.pi/2:
        return 1
    else:
        return 0


def difficulty_definition(x, y, path_diff, dis_force_x, dis_force_y, dis_force_z, vel_x, vel_y, diff_list):
    # Input:
    # x, y      :   robot position
    # path_diff :   static path difficulty
    
    force_diff = is_disturbed(dis_force_x, dis_force_y, dis_force_z)
    dir_diff = is_backward(vel_x, vel_y)
    tmp = path_diff.copy()
    tmp['dist'] = (tmp['x']-x)**2+(tmp['y']-y)**2
    path_diff = tmp.loc[tmp.dist==tmp['dist'].min(), 'difficulty'].tolist()[0]
    diff_short_describe = path_diff+str(force_diff)+str(dir_diff)
    diff_describe = 'path difficulty: '+ path_diff + '  force difficulty: '+ str(force_diff)+'  direction difficulty: '+ str(dir_diff)
    if diff_short_describe in diff_list:
        diff_num = diff_list[diff_short_describe]
    else:
        diff_num = len(diff_list)
        diff_list[diff_short_describe] = len(diff_list)
    #rospy.loginfo(diff_short_describe)    
    return diff_num, diff_describe, diff_list
    pass

############################feature extraction###############################################################
    

def calc_velocity(vel_x, vel_y, vel_z, del_low=True, threshold=0.05):
    
    vel = vel_x**2 + vel_y**2 + vel_z**2
    vel = vel**(1/2)
    if del_low and vel<threshold:
        return None
    else:
        return vel
    
def calc_force(fx, fy, fz):
    ret = fx**2 + fy**2 + fz**2
    ret = ret**(1/2)
    return ret

def calc_torque_xy(df):
    df['torque_xy'] = df['Mz']-df['DMz']
    return df

def calc_torque_turning_freq(history):
    tmp1 = np.array(history)[:-1]
    tmp2 = np.array(history)[1:]
    
    tmp3 = tmp2-tmp1
       
    
def calc_tremble(history, length):
    if len(history)<length:
        return -1
    tmp = history[['Mx', 'My', 'Mz', 'DMx', 'DMy', 'DMz']].copy()
    
    tmp = calc_torque_xy(tmp)
    tmp['tt_freq'] = tmp['torque_xy']
    tmp.loc[0, 'tt_freq'] = 0
    tmp1 = tmp['torque_xy'][:-1].reset_index(drop=True)
    tmp2 = tmp['torque_xy'][1:].reset_index(drop=True)
    tmp3 = tmp2-tmp1
    tmp3.index = tmp3.index+1
    
    tmp.loc[1:, 'tt_freq'] = tmp3
    
    tmp3 = tmp[tmp['tt_freq']!=0][['tt_freq']]
    tmp3 = tmp3.reset_index()
    tmp1 = tmp3['tt_freq'][:-1].reset_index()
    tmp2 = tmp3['tt_freq'][1:].reset_index()
    tmp4 = tmp1*tmp2
    tmp4.index += 1
    tmp3[0, 'tt_freq'] = 0
    
    tmp3.loc[1:, 'tt_freq'] = tmp4
    tmp3.index = tmp3['index']
    tmp.loc[tmp.tt_freq!=0, 'tt_freq'] = tmp3['tt_freq']
    
    ret = len(tmp.loc[tmp.tt_freq<0, ])
    del tmp
    del tmp1
    del tmp2
    del tmp3
    del tmp4
    return ret

def calc_deviation(front, left, right, c=0.1):
    front = float(front)
    left = float(left)
    right = float(right)
    
    dist = min([front, left, right])
    
    return  1/(dist+c)
    

    
def calc_feature_value(feature, vel_x, vel_y, vel_z, fx, fy, fz, del_low=True, threshold=0.05):
    if feature=='velocity':
        return calc_velocity(vel_x, vel_y, vel_z, del_low, threshold)
    elif feature=='force':
        return calc_force(fx, fy, fz)
 
###############################################################################################################
#################### some function for model building or calculation ##########################################
def ability_level_mapper(data, parameters):
            
    col, how, n_level, invert, v_max, v_min, imbalanced_data = parameters
    #rospy.loginfo([v_max, v_min, n_level])
    
    interval = (v_max- v_min)/n_level
    assert interval!=0, 'zero dividend'
    ret = int((data-v_min)/interval)
        
    if invert:
        ret = n_level-ret+1
    else:
        ret = ret+1
        
    return ret
        


def cMMGRM(theta, alpha, b, w , c):
    return math.e**(alpha*(w*theta-b+c))/(1+math.e**(alpha*(w*theta-b+c)))
    
def model(theta, alpha, b, w, cl, start=1):
    sum = start*1
    for c in cl:
        sum += cMMGRM(theta, alpha, b, w, c)
    return sum

def calc_theta_for_diff(avg_tested_feature,  alpha, b, w, cl, candidates):
    
    ca_ser = pd.Series(candidates)
    estimates = model(ca_ser, alpha, b, w, cl)
    comb = pd.concat([ca_ser, estimates], axis=1)
    comb.columns = ['ca', 'est']
    comb['est'] = comb['est'] - avg_tested_feature
    comb['est'] = comb['est']**2
    #ret = comb.loc[comb.est==comb['est'].min(), 'ca']
    #ret = ret.tolist()[0]
    #rospy.loginfo(comb)
    return comb

def select_theta_and_update(loss_diff, diff, avg_tested_feature, alpha, b, w, cl, candidates):
    loss_diff = loss_diff[loss_diff['diff']!=diff]
    new_diff = calc_theta_for_diff(avg_tested_feature, alpha, b, w, cl, candidates)
    new_diff['diff'] = diff
    loss_diff = pd.concat([loss_diff, new_diff])
    tmp = loss_diff.groupby(['ca']).sum().reset_index()
    ret = tmp.loc[tmp.est==tmp['est'].min(), 'ca']
    ret = ret.tolist()[0]
    return ret

def set_cl(n=19, start=-1, end=1, test_col_p=None):
    if test_col_p is None or test_col_p is False:
        if start<end:
            tmp = start
            start = end
            end = tmp
        return list(np.linspace(start, end, n))
    # if test column is segmented in different length, the c list should have the same distributiion
    else:
        cl = []
        lth = end-start
        # test_col_p denotes the percentage of each point in the whole scale
        # e.g. [0, 0.05, 0.10, ..., 1]
        l2 = test_col_p[-1]-test_col_p[0]
        
        
        for element in test_col_p:
            cl.append((element-test_col_p[0])*lth/l2+start)
        return cl.sort(reverse=True)
    
def build_model():
    pass

def load_params(path_type, features):
    path_type = str(path_type)
    
    # model params
    model_params = {}
    score_cat_params = {}
    cl = {}
    path = os.path.dirname(os.path.realpath(__file__))
    
    for feature in features:
        path1 = path+'/../data/irt_parameters_'+ feature +'.p'
        with open(path1, 'rb') as f:
            _, item_params = pickle.load(f)
            model_params[feature] = item_params
        path2 = path+'/../data/ability_mapper_'+ feature +'.p'
        with open(path2, 'rb') as f:
            score_cat_params[feature] = pickle.load(f)
        
        col, how, n_level, invert, v_max, v_min, imbalanced_data = score_cat_params[feature]
        cl[feature] = set_cl(n_level)
    path3 =path + '/../data/heika_path'+ path_type +'.csv'
    static_path_diff = pd.read_csv(path3)
    #rospy.loginfo(feature)
    
    return (model_params, score_cat_params, static_path_diff, cl)
         
#####################################################################################################################################   
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

def callback_deviation(data):
    pass

def user_performance():
    #rospy.init_node('robotrainer_heika_classification_based_user_performance')
    rospy.init_node('robotrainer_user_perfomance')
    
    ############################################################## init variables #################################################################################
    ###############################################################################################################################################################
    cols = 'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz', 'DFx', 'DFy', 'DFz', 'DMx', 'DMy', 'DMz' , 'Lx', 'Ly', 'Lz', 'Ax', 'Ay', 'Az' , 'front', 'left', 'right', 'pose.x', 'pose.y', 'pose.theta', 'leg_pose_x', 'leg_pose_y'
    record_history = pd.DataFrame()
    candidates = np.linspace(-4, 4, 1000)
    features = ['velocity', 'force', 'tremble', 'deviation']
    avg_dict = {}
    avg_dict['velocity'] = {}
    avg_dict['force'] = {}
    avg_dict['tremble'] = {}
    avg_dict['deviation'] = {}
    pubs = {}
    history_length = 100
    loss_diff = pd.DataFrame(np.array([]).reshape(0, 3), columns = ['diff', 'ca', 'est'])
    
    ###########################difficulty definition###################################
    diff_list = {}
    diff_list['line00'] = 1
    diff_list['line01'] = 4
    ###################################################################################
    path_type = rospy.get_param('~path_type')
    
    model_params,  score_cat_params, static_path_diff, cl = load_params(path_type, features)
    ###################publisher and subscriber ##########################################
    pubs['velocity'] = rospy.Publisher('ability/velocity', Float64, queue_size=10)
    pubs['force'] = rospy.Publisher('ability/force', Float64, queue_size=10)
    pubs['tremble'] = rospy.Publisher('ability/tremble', Float64, queue_size=10)
    pubs['deviation'] = rospy.Publisher('ability/deviation', Float64, queue_size=10)
    
    rospy.Subscriber("mobile_robot_pose", Pose2DStamped, callback_mobile_robot_pose)
    rospy.Subscriber("input_force", WrenchStamped, callback_input_force)
    rospy.Subscriber("disturbance_force", WrenchStamped, callback_disturbance_force)
    rospy.Subscriber("robot_command", Twist, callback_robot_command)
    #rospy.Subscriber("checkpoint", Marker, callback_checkpoint)
    #rospy.Subscriber("path_update", InteractiveMarkerUpdate, callback_path_update) # Why?
    rospy.Subscriber("odometry", Odometry, callback_odometry)
    rospy.Subscriber("checkpoint_info", String, callback_checkpoint_info)
    #rospy.Subscriber("legs_people", MarkerArray, callback_legs_people)
    
    rospy.Subsrciber("deviation", RobotrainerUserDeviation, callback_deviation)

    #Create your own structure and fill with data from the callbacks
    # Open questions:
    ## How should we synchronize data?
    ## How often should we call your algorithm?
    ## How should look your output structure?
    rate = rospy.Rate(10)
    calc_rate = 1
    pre_time = time.time()
    
    ############################################################### main procedure #################################################################################
    #################################################################################################################################################################
    while not rospy.is_shutdown():
        features = ['velocity', 'force', 'tremble', 'deviation']
        score_cate = {}
        alpha = {}
        b = {}
        w = {}
        feature_value = {}
        
        
        if pose_x is np.nan:
            rate.sleep()
            continue
        
        # load path difficulty and transform topic to record
        diff_num, diff_des, diff_list = difficulty_definition(pose_x, pose_y, static_path_diff, dis_force_x, dis_force_y, dis_force_z, vel_linear_x, vel_linear_y, diff_list)
        #rospy.loginfo(type(input_torque_x))
        curr = get_record([input_force_x, input_force_y, input_force_z, input_torque_x, input_torque_y, input_torque_z, dis_force_x, dis_force_y, dis_force_z, 
                           dis_torque_x, dis_torque_y, dis_torque_z, vel_linear_x, vel_linear_y, vel_linear_z, vel_angular_x, vel_angular_y, vel_angular_z,
                           dist_front, dist_left, dist_right, pose_x, pose_y, pose_theta, leg_position_x, leg_position_y], cols)
        record_history = update_history(curr, record_history, length=history_length)
        
        # calc feature values
        v = calc_velocity(vel_linear_x, vel_linear_y, vel_linear_z)
        feature_value['velocity'] = v
        f = calc_force(input_force_x, input_force_y, input_force_z)
        feature_value['force'] = f
        t = calc_tremble(record_history, history_length)
        feature_value['tremble'] = t
        d = calc_deviation(dist_front, dist_left, dist_right, c=0.2)
        feature_value['deviation'] = d
        
        # movement conditions
        if t==-1:
            features.remove('tremble')
        if None in [v, f, t, d]:
            rate.sleep()
            continue
        
        # transform feature value in score category
        for feature in features:
            p = model_params[feature]
            # fetch item params
            _, alpha[feature], b[feature], w[feature] = p.loc[p.difficulty==diff_num, ].values.tolist()[0]
            # transform into score category
            score_cate[feature] = ability_level_mapper(feature_value[feature], score_cat_params[feature])
            avg_dict[feature] = update_avg_dict(avg_dict[feature], diff_num, score_cate[feature])
            
            avg, count = avg_dict[feature][diff_num]
            theta = select_theta_and_update(loss_diff, diff_num, avg, alpha=alpha[feature], b=b[feature], cl=cl[feature], w=w[feature], candidates=candidates)
            pubs[feature].publish(theta)
        
        rate.sleep()
	
    rospy.spin()

if __name__ == '__main__':
    init_var()
    user_performance()
