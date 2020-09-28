#!/usr/bin/env python
from geometry_msgs.msg import Twist, WrenchStamped
from ipr_helpers.msg import Pose2DStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import InteractiveMarkerUpdate, Marker, MarkerArray
from robotrainer_deviation.msg import RobotrainerUserDeviation
from std_msgs.msg import Float64, Int64, ColorRGBA
import json
import roslib
import rospy
import yaml
import math
import tf
import geometry_msgs.msg
import time
import signal
import sys
import numpy as np

import actionlib
from iirob_led.msg import BlinkyAction, BlinkyGoal

from numpy.linalg import norm
from SubscriptionHandler import SubscriptionHandler
from PublisherHandler import PublisherHandler
from DataFormats import RobotrainerData
from DataFormats import Feature
from UserEvaluation import UserEvaluation

##########################################################################
def signal_handler(sig, frame):
    # data['time_standing'] = time_standing_total
    # data['time_training'] = time_training_total
    # with open('data.txt', 'w') as outfile:
    # json.dump(data, outfile)
    # print(numpy.std(data['input_force'])
    # print(numpy.average(data['input_force'])
    # print(numpy.average(data['deviation'])
    sys.exit(0)




class UserPerfomance:

    def __init__(self):
        self.ASC = True
        self.DESC = False
        self.fetched_frame = RobotrainerData
        self.time_standing = 0
        self.time_training = 0
        self.current_path_index = [0, 0, 0]
        self.try_number = rospy.get_param("/task/try")
        self.user_id = rospy.get_param("/task/user_id")
        self.task_id = rospy.get_param("/configure/task_id")
        self.pivot_points = rospy.get_param("/pivot_points")
        self.features = rospy.get_param("/configure/features")
        self.path_difficulties = rospy.get_param("/configure/path_difficulties")
        self.data = {}
        self.current_pivot_point = 0
        with open('../difficulty_01.yaml', 'r') as infile:
            self.difficulties = yaml.safe_load(infile)
        self.init_data()
        self.pivot_goal = BlinkyGoal(ColorRGBA(0, 0.4, 0, 0.4), 3, 0.1, 0.1, 0, 384, 0, False, False)
        self.close_to_pivot_goal = BlinkyGoal(ColorRGBA(0.4, 0.4, 0.4, 0.4), 2, 0.1, 0.1, 0, 384, 0, False, False)
        self.green = False
        self.yellow = False
        self.weights = self.difficulties.weight
        self.current_orientation = [0.0, 0.0]
        print("score " + str(self.data["score"].get_score()))

    def init_data(self):
        for f in self.features:
            self.data[f] = Feature()
        self.data["score"] = UserEvaluation()
        print("score " + str(self.data["score"].get_score()))
        for pd in self.path_difficulties:
            for f in self.features:
                self.data[pd + f] = Feature()
            self.data[pd + 'score'] = UserEvaluation()
        for fsd in self.difficulties["force_strength_difficulty"]:
            for f in self.features:
                self.data["fsd_" + str(fsd["id"]) + '_' + f] = Feature()
            self.data["fsd_" + str(fsd["id"]) + '_score'] = UserEvaluation()
        for dd in self.difficulties["direction_difficulty"]:
            for f in self.features:
                self.data["dd_" + str(dd["id"]) + '_' + f] = Feature()
            self.data["dd_" + str(dd["id"]) + '_score'] = UserEvaluation()
        for fdd in self.difficulties["force_direction_difficulty"]:
            for f in self.features:
                self.data["fdd_" + str(fdd["id"]) + '_' + f] = Feature()
            self.data["fdd_" + str(fdd["id"]) + '_score'] = UserEvaluation()
        self.data["difficulty"] = Feature()

        # for pd in self.path_difficulties:
        #    for fsd in self.difficulties["force_strength_difficulty"]:
        #         for dd in self.difficulties["direction_difficulty"]:
        #             for fdd in self.difficulties["force_direction_difficulty"]:
        #                 for f in self.features:
        #                     self.data[pd + "fsd_" + str(fsd["id"]) + '_' "dd_" + str(dd["id"]) +"_fdd_" + str(fdd["id"]) + '_' + f] = Feature()
        #                 self.data[pd + "fsd_" + str(fsd["id"]) + '_' "dd_" + str(dd["id"]) + "_fdd_" + str(fdd["id"]) + '_' + 'score'] = UserEvaluation()
        print("score " + str(self.data["score"].get_score()))

    def update_current_orientation(self):
        self.current_orientation = self.fetched_frame.velocity
        print(self.current_orientation)

    def current_orientation_deviation(self):
        v1 = np.array(self.current_orientation)
        v2 = np.array([rospy.get_param("/robotrainer/scenario/path/point"+ str(self.current_path_index[self.current_marker()]) + "/x"), rospy.get_param("/robotrainer/scenario/path/point" + str(self.current_path_index[self.current_marker()])+ "/y")])
        v1_u = v1 / np.linalg.norm(v1)
        v2_u = v2 / np.linalg.norm(v2)
        angle = (180 / np.pi) * np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
        if not np.isnan(angle):
            return angle
        else:
            return 0.0


    def current_force_angle_difficulty(self):
        for d in self.difficulties["force_direction_difficulty"]:
            if d["key"] > self.calc_virtual_force_angle():
                return d["id"]
        return 0
    def current_force_angle_difficulty_value(self):
        for d in self.difficulties["force_direction_difficulty"]:
            if d["key"] > self.calc_virtual_force_angle():
                return d["difficulty"]
        return 0
    def current_force_strength_difficulty(self):
        for d in self.difficulties["force_strength_difficulty"]:
            if d["key"] > self.calc_virtual_force():
                return d["id"]
        return 0
    def current_force_strength_difficulty_value(self):
        for d in self.difficulties["force_strength_difficulty"]:
            if d["key"] > self.calc_virtual_force():
                return d["difficulty"]
        return 0
    def current_direction_difficulty(self):
        if self.pivot_points[self.current_pivot_point]["direction"] == "FRONT":
            if rospy.get_param("/configure/y_reversed"):
                return 4
            else:
                return 0

        if self.pivot_points[self.current_pivot_point]["direction"] == "BACK":
            if rospy.get_param("/configure/y_reversed"):
                return 7
            else:
                return 3
        if self.pivot_points[self.current_pivot_point]["direction"] == "LEFT":
            if rospy.get_param("/configure/y_reversed"):
                return 5
            else:
                return 1
        if self.pivot_points[self.current_pivot_point]["direction"] == "RIGHT":
            if rospy.get_param("/configure/y_reversed"):
                return 6
            else:
                return 2
        if self.pivot_points[self.current_pivot_point]["direction"] == "END":
            return 0
        return 0
    def current_direction_difficulty_value(self):
        for d in self.difficulties["direction_difficulty"]:
            if d["key"] == self.current_direction_difficulty():
                return d["difficulty"]
        return 1

    def current_path_difficulty(self):
        if (rospy.has_param('/path_points_difficulty/point'+ str(self.current_path_index) +'/')):
            return rospy.get_param('/path_points_difficulty/point'+ str(self.current_path_index) +'/')
        return self.pivot_points[self.current_pivot_point]["pathdifficulty"]
    def current_path_difficulty_value(self):
        for d in self.difficulties["path_difficulty"]:
            if d["key"] == self.current_path_difficulty():
                return d["difficulty"]
        return 0

    def calculate_difficulty(self):
        #print("diff:" + str(self.current_direction_difficulty_value()  * self.current_path_difficulty_value() * self.current_force_angle_difficulty_value() * self.current_force_strength_difficulty_value()))
        return max(self.current_direction_difficulty_value()  * self.current_path_difficulty_value() * self.current_force_angle_difficulty_value() * self.current_force_strength_difficulty_value(), 1)

    def progress_current_pivot(self):
        if self.current_pivot_point + 1 >= len(self.pivot_points):
            return 1.0
        if self.is_current_order_asc():
            total = self.pivot_points[self.current_pivot_point + 1]["pathindex"] - self.pivot_points[self.current_pivot_point]["pathindex"]
            pro = self.current_path_index[self.current_marker()] - self.pivot_points[self.current_pivot_point]["pathindex"]
            if (float(pro) / float(total) > 0.8) and (float(pro) / float(total) < 0.9):
                self.yellow = True
        else:
            total = self.pivot_points[self.current_pivot_point]["pathindex"] - self.pivot_points[self.current_pivot_point + 1]["pathindex"]
            pro =  self.pivot_points[self.current_pivot_point]["pathindex"] - self.current_path_index[self.current_marker()]
            if (float(pro) / float(total) > 0.8) and (float(pro) / float(total) < 0.9):
                self.yellow = True



    # false if DESC true if ASC
    def is_current_order_asc(self):
        if self.current_pivot_point + 1 < len(self.pivot_points):
            current_pi = self.pivot_points[self.current_pivot_point]["pathindex"]
            next_pi = self.pivot_points[self.current_pivot_point + 1]["pathindex"]
            return current_pi < next_pi
        else:
            return True

    def current_marker(self):
        if self.pivot_points[self.current_pivot_point]["direction"] == "FRONT":
            return 0
        if self.pivot_points[self.current_pivot_point]["direction"] == "BACK":
            return 0
        if self.pivot_points[self.current_pivot_point]["direction"] == "LEFT":
            return 1
        if self.pivot_points[self.current_pivot_point]["direction"] == "RIGHT":
            return 2
        if self.pivot_points[self.current_pivot_point]["direction"] == "END":
            return 0

    def task_ended(self):
        return self.current_pivot_point == len(self.pivot_points) - 1

    def update_current_pivot_point(self):
        if self.current_pivot_point == len(self.pivot_points) - 1:
            return
        if self.is_current_order_asc() == self.ASC:
            if self.current_path_index[self.current_marker()] >= self.pivot_points[self.current_pivot_point + 1]["pathindex"]:
                print("green")
                self.send_green_leds()
                self.current_pivot_point += 1
        else:
            if self.current_path_index[self.current_marker()] <= self.pivot_points[self.current_pivot_point + 1]["pathindex"]:
                self.send_green_leds()
                self.current_pivot_point += 1

    def send_green_leds(self):
        self.green = True




    #########################################################################
    def calc_velocity(self):
        if self.fetched_frame.velocity_callback_number > 0:
            ret = [0.0, 0.0, 0.0]
            ret[0] = self.fetched_frame.velocity[0] / self.fetched_frame.velocity_callback_number
            ret[1] = self.fetched_frame.velocity[1] / self.fetched_frame.velocity_callback_number
            ret[2] = self.fetched_frame.velocity[2] / self.fetched_frame.velocity_callback_number
            return (ret[0] ** 2 + ret[1] ** 2 + ret[2] ** 2) ** 0.5
        else:
            return 0.0


    def calc_force(self, force):
        ret = 0.0
        for f in force:
            ret += f ** 2
        return ret ** 0.5


    def calc_input_force(self):
        if self.fetched_frame.input_force_callback_number > 0:
            ret = [0.0, 0.0 , 0.0]
            ret[0] = self.fetched_frame.input_force[0] / self.fetched_frame.input_force_callback_number
            ret[1] = self.fetched_frame.input_force[1] / self.fetched_frame.input_force_callback_number
            ret[2] = self.fetched_frame.input_force[2] / self.fetched_frame.input_force_callback_number
            return self.calc_force(ret)
        else:
            return 0.0

    def calc_virtual_force(self):
        if self.fetched_frame.disturbance_force_callback_number > 0:
            ret = [0.0, 0.0 , 0.0]
            ret[0] = self.fetched_frame.disturbance_force[0] / self.fetched_frame.disturbance_force_callback_number
            ret[1] = self.fetched_frame.disturbance_force[1] / self.fetched_frame.disturbance_force_callback_number
            ret[2] = self.fetched_frame.disturbance_force[2] / self.fetched_frame.disturbance_force_callback_number
            return self.calc_force(ret)
        else:
            return 0.0
    def calc_virtual_force_angle(self):
        if (self.fetched_frame.input_force_callback_number > 0) and (self.fetched_frame.disturbance_force_callback_number > 0):
            input_force = [0.0, 0.0, 0.0]
            input_force[0] = self.fetched_frame.input_force[0] / self.fetched_frame.input_force_callback_number
            input_force[1] = self.fetched_frame.input_force[1] / self.fetched_frame.input_force_callback_number
            input_force[2] = self.fetched_frame.input_force[2] / self.fetched_frame.input_force_callback_number
            virtual_force = [0.0, 0.0, 0.0]
            virtual_force[0] = self.fetched_frame.disturbance_force[0] / self.fetched_frame.disturbance_force_callback_number
            virtual_force[1] = self.fetched_frame.disturbance_force[1] / self.fetched_frame.disturbance_force_callback_number
            virtual_force[2] = self.fetched_frame.disturbance_force[2] / self.fetched_frame.disturbance_force_callback_number
            v1 = np.array(input_force)
            v2 = np.array(virtual_force)
            v1_u = v1 / np.linalg.norm(v1)
            v2_u = v2 / np.linalg.norm(v2)
            angle = (180 / np.pi) * np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
            if not np.isnan(angle):
                return angle
            else:
                return 0.0
        else:
            return 0.0

    def calc_average_input_force(self):
        return self.total_input_force / self.number_of_iterations


    def calc_average_velocity(self):
        return self.total_velocity / self.number_of_iterations


    # deviation after paper "Robot-Based Training for People With MildCognitive Impairment", April 2019
    def calc_deviation(self):
        if np.isclose(float(self.fetched_frame.deviation[self.current_marker()]), -1.0):
            return np.nan
        return float(self.fetched_frame.deviation[self.current_marker()])


    def is_robot_standing(self):
        return self.fetched_frame.input_force[0] == 0 and self.fetched_frame.input_force[1] == 0


    def update(self):
        #print(self.current_force_angle_difficulty())
        self.data["difficulty"].update((self.calculate_difficulty()))
        self.current_path_index = self.fetched_frame.path_index
        self.update_current_pivot_point()
        self.update_current_orientation()
        self.update_totals()
        self.update_by_path_difficulty()
        self.update_by_virtual_force_angle_difficulty()
        self.update_by_virtual_force_strength_difficulty()
        self.update_by_direction_difficulty()
        self.update_by_difficulty()
        self.progress_current_pivot()

    def get_data(self, feature):
        return self.data[feature].get_characteristic()

    def update_totals(self):
        #print(self.calc_velocity())
        self.data["velocity"].update(self.calc_velocity())
        self.data["deviation"].update(self.calc_deviation())
        self.data["input_force"].update(self.calc_input_force())
        self.data["orientation"].update(self.current_orientation_deviation())
        self.data["score"].eval_function(self.calculate_difficulty(), self.weights ,[self.calc_deviation(),self.current_orientation_deviation], self.calc_velocity())



    def update_by_path_difficulty(self):
        #print(self.calc_velocity())
        self.data[self.current_path_difficulty() + "velocity"].update(self.calc_velocity())
        self.data[self.current_path_difficulty() + "deviation"].update(self.calc_deviation())
        self.data[self.current_path_difficulty() + "input_force"].update(self.calc_input_force())
        self.data[self.current_path_difficulty() + "orientation"].update(self.current_orientation_deviation())
        self.data[self.current_path_difficulty() + "score"].eval_function(self.calculate_difficulty(),self.weights ,[self.calc_deviation(),self.current_orientation_deviation], self.calc_velocity())

    def update_by_virtual_force_angle_difficulty(self):

        self.data["fdd_" + str(self.current_force_angle_difficulty()) + "_velocity"].update(self.calc_velocity())
        self.data["fdd_" + str(self.current_force_angle_difficulty()) + "_deviation"].update(self.calc_deviation())
        self.data["fdd_" + str(self.current_force_angle_difficulty()) + "_input_force"].update(self.calc_input_force())
        self.data["fdd_" + str(self.current_force_angle_difficulty()) + "_orientation"].update(self.current_orientation_deviation())
        self.data["fdd_" + str(self.current_force_angle_difficulty()) + "_score"].eval_function(self.calculate_difficulty(),self.weights ,[self.calc_deviation(),self.current_orientation_deviation], self.calc_velocity())

    def update_by_virtual_force_strength_difficulty(self):
        #print(self.calc_velocity())
        self.data["fsd_" + str(self.current_force_strength_difficulty()) + "_velocity"].update(self.calc_velocity())
        self.data["fsd_" + str(self.current_force_strength_difficulty()) + "_deviation"].update(self.calc_deviation())
        self.data["fsd_" + str(self.current_force_strength_difficulty()) + "_input_force"].update(self.calc_input_force())
        self.data["fsd_" + str(self.current_force_strength_difficulty()) + "_orientation"].update(self.current_orientation_deviation())
        self.data["fsd_" + str(self.current_force_strength_difficulty()) + "_score"].eval_function(self.calculate_difficulty(),self.weights ,[self.calc_deviation(),self.current_orientation_deviation], self.calc_velocity())

    def update_by_direction_difficulty(self):
        #print(self.calc_velocity())
        self.data["dd_" + str(self.current_direction_difficulty()) + "_velocity"].update(self.calc_velocity())
        self.data["dd_" + str(self.current_direction_difficulty()) + "_deviation"].update(self.calc_deviation())
        self.data["dd_" + str(self.current_direction_difficulty()) + "_input_force"].update(self.calc_input_force())
        self.data["dd_" + str(self.current_direction_difficulty()) + "_orientation"].update(self.current_orientation_deviation())
        self.data["dd_" + str(self.current_direction_difficulty()) + "_score"].eval_function(self.calculate_difficulty(),self.weights ,[self.calc_deviation(),self.current_orientation_deviation], self.calc_velocity())

    def update_by_difficulty(self):
        path_dif = self.current_path_difficulty()
        fsd = self.current_force_strength_difficulty()
        dd = self.current_direction_difficulty()
        fad = self.current_force_angle_difficulty()
        #print(self.calc_velocity())

        if path_dif + "fsd_" + str(fsd) + '_' "dd_" + str() + "_fdd_" + str(dd) + "_velocity" in self.data:
            self.data[path_dif + "fsd_" + str(fsd) + '_' "dd_" + str(dd) + "_fdd_" + str(fad) + "_velocity"].update(self.calc_velocity())
        else:
            self.data[path_dif + "fsd_" + str(fsd) + '_' "dd_" + str(dd) + "_fdd_" + str(fad) + "_velocity"] = Feature()
            self.data[path_dif + "fsd_" + str(fsd) + '_' "dd_" + str(dd) + "_fdd_" + str(fad) + "_velocity"].update(self.calc_velocity())

        if path_dif + "fsd_" + str(fsd) + '_' "dd_" + str(dd) + "_fdd_" + str(fad) + "_deviation" in self.data:
            self.data[path_dif + "fsd_" + str(fsd) + '_' "dd_" + str(dd) + "_fdd_" + str(fad) + "_deviation"].update(self.calc_deviation())
        else:
            self.data[path_dif + "fsd_" + str(fsd) + '_' "dd_" + str(dd) + "_fdd_" + str(fad) + "_deviation"] = Feature()
            self.data[path_dif + "fsd_" + str(fsd) + '_' "dd_" + str(dd) + "_fdd_" + str(fad) + "_deviation"].update(self.calc_deviation())

        if path_dif + "fsd_" + str(fsd) + '_' "dd_" + str(dd) + "_fdd_" + str(fad) + "_input_force" in self.data:
            self.data[path_dif + "fsd_" + str(fsd) + '_' "dd_" + str(dd) + "_fdd_" + str(fad) + "_input_force"].update(self.calc_input_force())
        else:
            self.data[path_dif + "fsd_" + str(fsd) + '_' "dd_" + str(dd) + "_fdd_" + str(fad) + "_input_force"] = Feature()
            self.data[path_dif + "fsd_" + str(fsd) + '_' "dd_" + str(dd) + "_fdd_" + str(fad) + "_input_force"].update(self.calc_input_force())


        if path_dif + "fsd_" + str(fsd) + '_' "dd_" + str(fsd) + "_fdd_" + str(dd) + "_orientation" in self.data:
            self.data[path_dif + "fsd_" + str(fsd) + '_' "dd_" + str(fsd) + "_fdd_" + str(dd) + "_orientation"].update(self.current_orientation_deviation())
        else:
            self.data[path_dif + "fsd_" + str(fsd) + '_' "dd_" + str(fsd) + "_fdd_" + str(dd) + "_orientation"] = Feature()
            self.data[path_dif + "fsd_" + str(fsd) + '_' "dd_" + str(fsd) + "_fdd_" + str(dd) + "_orientation"].update(self.current_orientation_deviation
                                                                                                                                                                                                                                     ())

        if path_dif + "fsd_" + str(fsd) + '_' "dd_" + str(dd) + "_fdd_" + str(fad) + "_score" in self.data:
            self.data[path_dif + "fsd_" + str(fsd) + '_' "dd_" + str(dd) + "_fdd_" + str(fad) + "_score"].eval_function(self.calculate_difficulty(),[1],[self.calc_deviation()], self.calc_velocity())
        else:
            self.data[path_dif + "fsd_" + str(fsd) + '_' "dd_" + str(dd) + "_fdd_" + str(fad) + "_score"] = UserEvaluation()
            self.data[path_dif + "fsd_" + str(fsd) + '_' "dd_" + str(dd) + "_fdd_" + str(fad) + "_score"].eval_function(self.calculate_difficulty(),[1],[self.calc_deviation()], self.calc_velocity())


    def create_yaml_from_task(self):
        rospy.loginfo("")
        yaml_dump = {"velocity": {}, "deviation": {}, "input_force": {}, "score" : {}, "orientation" : {}}
        for feat in self.features:
            yaml_dump[feat]["general"] = self.data[feat].get_characteristic()
        yaml_dump["score"]["general"] = self.data["score"].get_score()
        for pdiff in self.path_difficulties:
            for feat in features:
                yaml_dump[feat][pdiff] = self.data[pdiff + feat].get_characteristic()
            yaml_dump["score"][pdiff] = self.data[pdiff + "score"].get_score()
        for fsd in self.difficulties["force_strength_difficulty"]:
            for feat in self.features:
                yaml_dump[feat]["fsd_" + str(fsd["id"])] = self.data["fsd_" + str(fsd["id"]) + '_' + feat].get_characteristic()
            yaml_dump["score"]["fsd_" + str(fsd["id"])] = self.data["fsd_" + str(fsd["id"]) + '_' + "score"].get_score()
        for dd in self.difficulties["direction_difficulty"]:
            for feat in self.features:
                yaml_dump[feat]["dd_" + str(dd["id"])] = self.data["dd_" + str(dd["id"]) + '_' + feat].get_characteristic()
            yaml_dump["score"]["dd_" + str(dd["id"])] = self.data["dd_" + str(dd["id"]) + '_' + "score"].get_score()
        for fdd in self.difficulties["force_direction_difficulty"]:
            for feat in self.features:
                yaml_dump[feat]["fdd_" + str(fdd["id"])] = self.data["fdd_" + str(fdd["id"]) + '_' + feat].get_characteristic()
            yaml_dump["score"]["fdd_" + str(fdd["id"])] = self.data["fdd_" + str(fdd["id"]) + '_' + "score"].get_score()

        for pd in self.path_difficulties:
            for fsd in self.difficulties["force_strength_difficulty"]:
                for dd in self.difficulties["direction_difficulty"]:
                    for fdd in self.difficulties["force_direction_difficulty"]:
                        for feat in self.features:
                            if pd + "fsd_" + str(fsd["id"]) + '_' "dd_" + str(dd["id"]) + "_fdd_" + str(fdd["id"]) + '_' + feat in self.data:
                                yaml_dump[feat][pd + "_fsd_" + str(fsd["id"]) + '_' "dd_" + str(dd["id"]) + '_' + "_fdd_" + str(fdd["id"])] = self.data[pd + "fsd_" + str(fsd["id"]) + '_' "dd_" + str(dd["id"]) + "_fdd_" + str(fdd["id"]) + '_' + feat].get_characteristic()
                        if pd + "fsd_" + str(fsd["id"]) + '_' "dd_" + str(dd["id"]) + "_fdd_" + str(fdd["id"]) + '_' + "score" in self.data:
                            yaml_dump["score"][pd + "_fsd_" + str(fsd["id"]) + '_' "dd_" + str(dd["id"]) + '_' + "_fdd_" + str(fdd["id"])] = self.data[pd + "fsd_" + str(fsd["id"]) + '_' "dd_" + str(dd["id"]) + "_fdd_" + str(fdd["id"]) + '_' + "score"].get_score()
        yaml_dump["difficulty"] = self.data["difficulty"].get_characteristic()
        yaml_dump["task_id"] = self.task_id
        yaml_dump["user_id"] = self.user_id
        yaml_dump["difficulty_id"] = self.difficulties["id"]
        yaml_dump["try"] = self.try_number
        yaml_dump["standing_time"] = self.time_standing
        yaml_dump["training_time"] = self.time_training

        with open('../result_' + self.task_id + '_' + self.user_id + '_' + str(self.try_number) +'.yml', 'w') as outfile:
            yaml.dump(yaml_dump, outfile)


if __name__ == '__main__':
    rospy.init_node('robotrainer_classic_user_perfomance')
    client = actionlib.SimpleActionClient('/rosy_test/leds_rectangle/blinky', BlinkyAction)
    client.wait_for_server()
    goal = BlinkyGoal(ColorRGBA(0.7, 0, 0, 0.8), 10, 0.1, 0.1, 0, 384, 0, False, False)
    # Fill in the goal here
    client.send_goal(goal)


    signal.signal(signal.SIGINT, signal_handler)
    up = UserPerfomance()

    sh = SubscriptionHandler()
    ph = PublisherHandler()
    pub_total_time_training = rospy.Publisher('total/time/training', Float64, queue_size=10)
    pub_total_time_standing = rospy.Publisher('total/time/standing', Float64, queue_size=10)
    path_difficulties = rospy.get_param("/configure/path_difficulties")
    features = rospy.get_param("/configure/features")
    rate = rospy.Rate(20)
    wrote_file = False
    start_time = time.time()
    while not rospy.is_shutdown():
        up.fetched_frame = sh.pull_data()
        standing = up.is_robot_standing()
        if up.is_robot_standing():
            pass
           # print('standing')
        else:
            #print("current=" + str(up.current_path_index[0]) + " percent=" + str( (up.progress_current_pivot())) + "total=" + str((up.current_path_index[0] *384.0)/557.0))
            #goal1 = BlinkyGoal(ColorRGBA(0, 0, 0.8, 0.3), 1, 0.1, 0.1, 0,int(min( ((up.progress_current_pivot() *384.0)), 384.0)), 0, False, False)

               # client.send_goal(goal1)

            if not up.task_ended():
                up.update()
                if up.green:
                    client.send_goal(up.pivot_goal)
                elif up.yellow:
                    client.send_goal(up.close_to_pivot_goal)
                else:
                    pass
                up.green = False
                up.yellow = False
        for f in features:
            ph.update_single_pub(f, up.get_data(f))
        for pd in path_difficulties:
            for f in features:
                ph.update_single_pub(pd + f, (up.get_data(pd + f)))
        ph.update_score_pub(up.data["score"].get_score())
        ph.update_diff_pub(up.data["difficulty"].get_characteristic())
        if up.task_ended() and not wrote_file:
            print("done")
            up.create_yaml_from_task()
            wrote_file = True
        ph.publish()
        sh.reset()
        if standing:
            up.time_standing += time.time() - start_time
        # standing time wont be recorded if the attempt hasent started yet
        elif (up.time_standing > 0.0) and (not up.task_ended()):
            up.time_training += time.time() - start_time
        start_time = time.time()
        pub_total_time_training.publish(up.time_training)
        pub_total_time_standing.publish(up.time_standing)
        rate.sleep()
    # rate = rospy.Rate(10)
    # pretime = time.time()
    #
    #     if input_force_x==0 and input_force_y==0: # sometimes Z variables are off
    #         if stopped == False:
    #             time_standing_start = time.time()
    #         stopped = True
    #         rate.sleep()
    #         continue
    #     updates += 1
    #     if stopped == True:
    #         time_standing_total += time.time() - time_standing_start
    #     time_training_total = time.time() - time_standing_total - pretime
    #     stopped = False
    #     total_input_force += calc_input_force()
    #     total_velocity += calc_velocity(vel_linear_x, vel_linear_y, vel_linear_z)
    #     reset_force_values()
    #     pub_total_force.publish(total_input_force)
    #     total_deviation += calc_deviation(dist_front, dist_left, dist_right)
    #     data['velocity'].append(calc_velocity(vel_linear_x, vel_linear_y, vel_linear_z))
    #     data['deviation'].append(calc_deviation(dist_front, dist_left, dist_right))
    #     data['input_force'].append(calc_force(input_force_x, input_force_y,input_force_z))
    #     rate.sleep()
