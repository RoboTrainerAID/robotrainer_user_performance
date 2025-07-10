#!/usr/bin/env python

import sys
import roslib
roslib.load_manifest('diagnostic_updater')
import rospy
import rospkg

import actionlib
import diagnostic_updater
from iirob_led.msg import BlinkyAction, BlinkyGoal
from std_msgs.msg import String, ColorRGBA

import dynamic_reconfigure.server
from robotrainer_study_automatic_assessment.cfg import UserStudyManagerConfig

from std_srvs.srv import Trigger


class UserStudyManager:

    def __init__(self):
        rospy.init_node("user_study_manager")
        
        self.diagnostic = diagnostic_updater.Updater()
        self.diagnostic.add("User Study Manager", self.diagnostics_callback)
        self.diagnostic.setHardwareID("User_Study_Manager")
        self.diagnostic.broadcast(0, "Initializing User Study Manager")

        self.frequency = rospy.get_param("~frequency", 1)
        self.study_name = rospy.get_param("~study_name", "")
        self.task_ids = rospy.get_param("~task_ids", [None])
        self.user_id_prefix = rospy.get_param("~user_id_prefix", "")
        self.user_id_length = rospy.get_param("~user_id_length", 3)
        self.output_string_separator = rospy.get_param("~output_string_separator", "")
        self.use_led_output = rospy.get_param("~user_id_prefix", "")

        self.manager_status_file = rospy.get_param("~manager_status_file", None)
        self.data_set_from_file = False
        if (self.manager_status_file is None):
            self.manager_status_file = "/user_study_manager.status"
            rospy.logwarn("Study Manager uses default status file '{}'".format(self.manager_status_file))

        pkg_dir = rospkg.RosPack().get_path("robotrainer_study_automatic_assessment")
        self.manager_status_file = pkg_dir + self.manager_status_file
        rospy.logwarn("Study Manager status file: {}".format(self.manager_status_file))

        study_name_from_config = self.study_name
        self.load_study_manager_status_file()
        if study_name_from_config != self.study_name:
            rospy.logwarn("Study Manager status file is from the wrong study. Using default params. The old status file will be overwritten!")
            self.data_set_from_file = False
            self.study_name = study_name_from_config
        
        # initialize study variables
        if not self.data_set_from_file:
            self.user_id = -1
            self.task_id = "END"
            self.trial = -1

        self.trial_changed = False
        self.task_changed = False
        self.user_id_changed = False

        self.format_user_id = '{UserID:0' + str(self.user_id_length) + 'd}'
        self.format_string = (self.study_name + self.output_string_separator + self.user_id_prefix + self.format_user_id +
                              self.output_string_separator + '{TaskID}' + '-' + '{TrialNr}')
        self.task_record_format = self.format_string
        rospy.loginfo("The status string will have format: " + self.format_string)
        rospy.logwarn("Study status: " + self.format_string.format(
            UserID=self.user_id, TaskID=self.task_id, TrialNr=self.trial))

        # initialize publishers and action clients
        self.status_publisher_string = rospy.publisher = rospy.Publisher(
            "~study_status", String, queue_size=1)
        
        self.rosbag_feedback_sub = rospy.Subscriber("/begin_write", String, self.rosbag_feedback_callback)

        self.sync_srv = rospy.ServiceProxy("/rt2_sca_sync_node/send_sync_signal_1s", Trigger)
        self.deviation_reset = rospy.ServiceProxy("/robotrainer_deviation/reset", Trigger)

        self.led_client = actionlib.SimpleActionClient(
          '/leds_rectangle/blinky', BlinkyAction)
        self.led_goal = None
        if self.led_client.wait_for_server(rospy.Duration(1)):
            self.led_goal = BlinkyGoal(ColorRGBA(0.8, 1.0, 0, 0.8), 10, 0.1, 0.1, 0, 0, 0, False, False)
            self.led_client.send_goal(self.led_goal)
        else:
            rospy.logerr("LED Server not found therefore it will not be used!")
            self.led_client = None

        # initialize dynamic_reconfigure server
        self.first_reconfigure_callback = True
        self.dyn_cfg_srv = dynamic_reconfigure.server.Server(
          UserStudyManagerConfig, self.reconfigure_callback)

        rospy.Timer(rospy.Duration(1/self.frequency), self.timer_callback)

        self.diagnostic.force_update()
        

    def timer_callback(self, event): 
        self.diagnostic.update()

        if self.user_id == -1 or self.trial == -1:
            rospy.logerr_throttle(5, "User_id or Trial is not set therefore the study_status is not published!")
            return
        
        self.status_publisher_string.publish(
            self.format_string.format(
            UserID=self.user_id, TaskID=self.task_id, TrialNr=self.trial))
            
        if self.trial_changed or self.task_changed or self.user_id_changed:
            self.write_study_manager_status_file()
            
        if self.trial_changed:
            # TODO(Denis): make this parameterizable
            rospy.logwarn("Waiting before triggering sync services")
            rospy.sleep(rospy.Duration(2))
            rospy.logwarn("Triggering sync services")
            # resp = self.sync_srv()
            # if not resp.success:
            #     rospy.logerr("Sync service responded with error...")
            # resp = self.deviation_reset()
            # if not resp.success:
            #     rospy.logerr("Deviation reset service responded with error...")
            
            self.trial_changed = False
            
        if self.task_changed:
            self.task_changed = False
            
        if self.user_id_changed:
            self.user_id_changed = False
    

    def rosbag_feedback_callback(self, message):
        # TODO(denis): Check if there is right thing started
        rospy.loginfo("Received start for the bag file {}.".format(message.data))


    def diagnostics_callback(self, stat):
        if self.user_id == -1 or self.trial == -1:
            stat.summary(2, "UserID or Trial-Nr. is not set!")
        else:
            stat.summary(0, "Study status is published!")

        stat.add("Study name", self.study_name)
        stat.add("Task IDs", self.task_ids)
        stat.add("Current User", self.format_user_id.format(UserID=self.user_id))
        stat.add("Current Task", self.task_id)
        stat.add("Current Trial", self.trial)
                            
        return stat


    def reconfigure_callback(self, config, level):
        if not self.first_reconfigure_callback:
            next_trial = self.trial
            if config.next_trial:
                if (self.trial == -1):
                    next_trial = 1
                else:
                    next_trial += 1
            else:
                try:
                    if int(config.trial) >= 0:
                        next_trial = int(config.trial)
                except Exception as e:
                    rospy.logerr(e)

            next_task = self.task_id
            if config.next_task:
                next_id = (self.task_ids.index(self.task_id)+1) % len(self.task_ids)
                next_task = self.task_ids[next_id]
                next_trial = 1
            elif config.task_id in self.task_ids:
                next_task = config.task_id
            else:
                rospy.logerr("TaskID: {TaskID} is not in the task list! \n \
                            Defined IDs are {TaskIDs}" \
                            .format(TaskID=config.task_id, TaskIDs=self.task_ids))

            next_user_id = self.user_id
            if config.next_user:
                if (self.user_id == -1):
                    next_user_id = 0
                else:
                    next_user_id += 1
                next_trial = 1
                next_task = self.task_ids[0]
            else:
                try:
                    user_id = int(config.user_id)
                    if (user_id < (pow(10, self.user_id_length))):
                        next_user_id = user_id
                    else:
                        rospy.logerr("UserID: {UserID} too large! \n \
                                    Maximal UserID is {MaxUserIDs}" \
                                    .format(UserID=config.user_id, MaxUserIDs=(pow(10, self.user_id_length) - 1)))
                except Exception as e:
                    rospy.logerr(e)
                    
            if (self.trial != next_trial):
                self.trial = next_trial
                self.trial_changed = True
                
            if (self.task_id != next_task):
                self.task_id = next_task
                self.task_changed = True
                
            if (self.user_id != next_user_id):
                self.user_id = next_user_id
                self.user_id_changed = True

        config.next_trial = False
        config.trial = str(self.trial)
        config.next_task = False
        config.task_id = self.task_id
        config.next_user = False
        config.user_id = self.format_user_id.format(UserID=self.user_id)
        
        self.first_reconfigure_callback = False
        if not (self.led_client is None):
            self.led_client.send_goal(self.led_goal)

        return config

    
    def write_study_manager_status_file(self):
        file = open(self.manager_status_file, 'w')
        file.writelines([self.study_name + "\n",
                         self.format_user_id.format(UserID=self.user_id) + "\n",
                         self.task_id + "\n",
                         str(self.trial) + "\n"])
        file.close()

    def load_study_manager_status_file(self):
        try:
            with open(self.manager_status_file, 'r') as file:
                lines = file.readlines()
                if len(lines) >= 4:
                    self.study_name = lines[0].strip()
                    self.user_id = int(lines[1].strip())
                    self.task_id = lines[2].strip()
                    self.trial = int(lines[3].strip())
                    self.data_set_from_file =  True
                    rospy.loginfo("Manager status file successfully read and variables updated.")
                else:
                    rospy.logwarn("Manager status file does not contain enough data. Using default parameters.")
        except Exception as e:
            rospy.logwarn("Failed to read manager status file: {}. Using default parameters.".format(e))


def main(args):
    user_study_manager = UserStudyManager()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
