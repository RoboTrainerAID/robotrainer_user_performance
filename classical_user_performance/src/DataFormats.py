import numpy as np

class RobotrainerData:
    def __init__(self):
        self.input_force = [0.0, 0.0, 0.0]
        self.input_force_callback_number = 0
        self.disturbance_force = [0.0, 0.0, 0.0]
        self.disturbance_torque = [0.0, 0.0, 0.0]
        self.disturbance_force_callback_number = 0
        self.deviation = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.velocity_callback_number = 0
        self.mobil_robot_pose = [0.0, 0.0, 0.0]
        self.path_index = [0, 0, 0]


class Feature:
    def __init__(self):
        self.total = 0.0
        self.average = 0.0
        self.std_deviation = 0.0
        self.values = []

    def get_characteristic(self):
        return [self.total, float(self.average), float(self.std_deviation)]

    def update(self, data):
        self.values.append(data)
        if not np.isnan(data):
            self.total += data
        self.std_deviation = np.nanstd(self.values)
        self.average = np.nanmean(self.values)