import rospy
import numpy as np

class UserEvaluation:

    def __init__(self):
        self.score = 0.0
        if rospy.has_param("/base/twist_controller/FTSController/x_max_vel"):
            self.x_max_vel = rospy.get_param("/base/twist_controller/FTSController/x_max_vel")
        else:
            self.x_max_vel = 1.1
        if rospy.has_param("/base/twist_controller/FTSController/y_max_vel"):
            self.y_max_vel = rospy.get_param("/base/twist_controller/FTSController/y_max_vel")
        else:
            self.y_max_vel = 1.1
        self.max_vel = min((self.x_max_vel ** 2 + self.y_max_vel ** 2) ** 0.5, 1.1)

    def eval_diff_function(self,diff, weights, values):
        sum = 0.0
        for i in range(0,len(weights)):
            sum += weights[i] * values[i]
          #  weighted_values = np.array(weights) * np.array(values)
          #  with_difficulty = (1 / 1 ** 2) * np.array(values)
           # return float(np.sum(with_difficulty))
        if not np.isnan(sum):
            return (1/diff ) * sum
        else:
            return 0.0

    def eval_function(self, diff, weights, values, vel):
        self.score += self.eval_diff_function(diff, weights, values) * (vel / self.max_vel)


    def get_score(self):
        return self.score