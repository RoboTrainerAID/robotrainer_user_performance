#!/usr/bin/env python
from robotrainer_deviation.msg import RobotrainerUserDeviation
from classical_user_performance.msg import FeatureCharacteristics
from std_msgs.msg import Float64, Int64
import rospy

class PublisherHandler:
    def __init__(self):
        self.path_difficulties = rospy.get_param("/configure/path_difficulties")
        self.features = rospy.get_param("/configure/features")
        self.features_pub = {}
        self.features_pub_data = {}
        self.init_features_pub()
        self.score = 0
        self.diff = [0.0,0.0,0.0]

    def init_features_pub(self):
        self.pub_score = rospy.Publisher('/classical_user_performance/score/', Float64, queue_size=10)
        self.pub_diff = rospy.Publisher('/classical_user_performance/difficulty/', FeatureCharacteristics, queue_size=10)
        for f in self.features:
            self.features_pub[f] = rospy.Publisher('/classical_user_performance/total/' + f, FeatureCharacteristics, queue_size=10)
            self.features_pub_data[f] = [0.0, 0.0, 0.0]
        for pd in self.path_difficulties:
            for f in self.features:
                self.features_pub[pd + f] = rospy.Publisher('/classical_user_performance/' + pd + "/" + f, FeatureCharacteristics, queue_size=10)
                self.features_pub_data[pd + f] = [0.0, 0.0, 0.0]
    def update_single_pub(self, f, values):
        self.features_pub_data[f] = values

    def update_score_pub(self, values):
        self.score = values

    def update_diff_pub(self, values):
        self.diff = values

    def publish(self):
        self.pub_score.publish(self.score)
        msg = FeatureCharacteristics()
        msg.total = self.diff[0]
        msg.average = self.diff[1]
        msg.standard_deviation = self.diff[2]
        self.pub_diff.publish(msg)
        for f in self.features_pub:
            msg = FeatureCharacteristics()
            msg.total = self.features_pub_data[f][0]
            msg.average = self.features_pub_data[f][1]
            msg.standard_deviation = self.features_pub_data[f][2]
            self.features_pub[f].publish(msg)



