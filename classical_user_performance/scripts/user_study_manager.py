#!/usr/bin/env python

import sys
import roslib
roslib.load_manifest('diagnostic_updater')
import rospy

import actionlib
import diagnostic_updater
from iirob_led.msg import BlinkyAction, BlinkyGoal
from std_msgs.msg import String, ColorRGBA

import dynamic_reconfigure.server
from classical_user_performance.cfg import UserStudyManagerConfig

from std_srvs.srv import Trigger


class UserStudyManager:

    def __init__(self):
        rospy.init_node("user_study_manager")
        
        self.callback_first_time = True

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
        # TODO: add here possibility to use custom list of user ids
        # self.user_id_list = rospy.get_param("~user_id_list", [None])
        # TODO: add here maximal number of trials for each task - could be nice used
        # self.num_trials = rospy.get_param("~num_trials", [None])
        
        self.stop_on_trial_change = rospy.get_param("~stop_on_trial_change", False)
        self.stop_on_task_change = rospy.get_param("~stop_on_task_change", False)
        self.stop_on_user_id_change = rospy.get_param("~stop_on_user_id_change", False)
        
        self.manager_status_file = rospy.get_param("~manager_status_file", None)
        self.data_set_from_file = False
        if (self.manager_status_file is None):
            self.manager_status_file = "./study_manager.status"
            rospy.logwarn("Study Manager uses default status file '{}'".format(self.manager_status_file))
        try:
            file = open(self.manager_status_file, "r")
            lines = file.readlines()
            if (self.study_name == line[0]):
                self.user_id = int(line[1])
                self.task_id = line[2]
                self.trial = int(line[3])
                self.data_set_from_file = True
            else:
                rospy.logwarn("Study Manager status file is from the wrong study. Using default params. The old status file will be overwritten!")
        except:
            rospy.logwarn("Manager status file not found, using default start parameters")
        self.external_status_files = rospy.get_param("~external_status_files", None)
        #self.external_status_files = {}
        #if not (ext_status_files is None):
            #for elem in ext_status_files:
                #self.external_status_files.update(elem)
        self.update_external_status_on_trial_change = rospy.get_param("~update_external_status_on_trial_change", {})
        self.update_external_status_on_task_change = rospy.get_param("~update_external_status_on_task_change", {})
        
        # initialize study variables
        if self.study_name:
          self.study_name += self.output_string_separator

        self.user_id = -1
        self.task_id = ""
        self.trial = -1
        self.publish_status_and_record = False
        
        self.trial_changed = False
        self.task_changed = False
        self.user_id_changed = False

        self.format_user_id = '{UserID:0' + str(self.user_id_length) + 'd}'
        self.format_string = (self.study_name + self.user_id_prefix + self.format_user_id +
                              self.output_string_separator + '{TaskID}' + '-' + '{TrialNr}')
        #self.task_record_format = (self.study_name + self.user_id_prefix + self.format_user_id +
                                   #self.output_string_separator + '{TaskID}\n')
        self.task_record_format = self.format_string
        rospy.loginfo("The status string will have format: " + self.format_string)

        # initialize publishers and action clients
        self.status_publisher_string = rospy.publisher = rospy.Publisher(
            "~study_status", String, queue_size=1)
        #self.status_publisher = rospy.publisher = rospy.Publisher(
            #"~study", StudyStatus, queue_size=1)
        
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
        if self.publish_status_and_record:
            if self.user_id != -1 and self.trial != -1:
                self.status_publisher_string.publish(
                  self.format_string.format(
                    UserID=self.user_id, TaskID=self.task_id, TrialNr=self.trial))
                  
                if self.trial_changed or self.task_changed or self.user_id_changed:
                    self.write_manager_status_file()
                  
                if self.trial_changed:
                    #if not self.stop_on_trial_change:
                        #self.update_command_from_list_and_call(self.update_external_status_on_trial_change,
                                                            #str(self.trial-1),
                                                            #"STOP")
                        #rospy.sleep(0.5)
                    
                    self.update_command_from_list_and_call(self.update_external_status_on_trial_change,
                                                           str(self.trial),
                                                           "START")
                    
                    # TODO(Denis): make this parameterizable
                    rospy.logwarn("Waiting before triggering sync services")
                    rospy.sleep(rospy.Duration(2))
                    rospy.logwarn("Triggering sync services")
                    resp = self.sync_srv()
                    if not resp.success:
                        rospy.logerr("Sync service responded with error...")
                    resp = self.deviation_reset()
                    if not resp.success:
                        rospy.logerr("Deviation reset service responded with error...")
                    
                    self.trial_changed = False
                  
                if self.task_changed:
                    self.update_command_from_list_and_call(self.update_external_status_on_task_change,
                                                           self.task_id,
                                                           "START")
                    self.task_changed = False
                    
                if self.user_id_changed:
                    self.user_id_changed = False
                    
            else:
                rospy.logerr_throttle(5, "User_id or Trial is not set therefore the study_status is not published!")
                
        else:
            if self.trial_changed:
                self.trial_changed = False
                
            if self.task_changed:
                # TODO: this is not logically correct, since task ID will also be triggered is published goes off,
                # It will work for now
                # I should probably set some running flag
                previous_id = self.task_ids.index(self.task_id)-1
                if (previous_id == -1):
                    previous_id = len(self.task_ids)-1
                self.update_command_from_list_and_call(self.update_external_status_on_task_change,
                                                       self.task_ids[previous_id],
                                                           "STOP")
                self.task_changed = False
                
            if self.user_id_changed:
                self.user_id_changed = False

        self.diagnostic.update()
        
    def update_command_from_list_and_call(self, update_dict, update_key, command):
        call_list = []
        if "all" in update_dict.keys():
            call_list.extend(update_dict["all"])
        if update_key in update_dict.keys():
            call_list.extend(update_dict[update_key])
        self.write_status_files(call_list, command)
        

    def rosbag_feedback_callback(self, message):
        # TODO(denis): Check if there is right thing started
        rospy.loginfo("Received start for the bag file {}.".format(message.data))


    def diagnostics_callback(self, stat):
        if not self.publish_status_and_record:
            stat.summary(1, "Study status is not published!")
        elif self.user_id == -1 or self.trial == -1:
            stat.summary(2, "UserID or Trial-Nr. is not set!")
        else:
            stat.summary(0, "Study status is published!")

        stat.add("Study name", self.study_name)
        stat.add("Task IDs", self.task_ids)
        stat.add("Current User", self.format_user_id.format(UserID=self.user_id))
        stat.add("Current Task", self.task_id)
        stat.add("Current Trial", self.trial)
        
        if not (self.external_status_files is None):
            for key in self.external_status_files.keys():
                stat.add("Process '{}'".format(key), self.read_pid_status(key))
                            
        return stat


    def reconfigure_callback(self, config, level):
        if not (self.first_reconfigure_callback and self.data_set_from_file):
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

            new_publish_status = self.publish_status_and_record
            if (self.stop_on_trial_change and self.trial_changed or
                self.stop_on_task_change and self.task_changed or
                self.stop_on_user_id_change and self.user_id_changed or
                self.callback_first_time):
                new_publish_status = False
                self.callback_first_time = False
            else:
                if (self.publish_status_and_record != config.publish_status_and_record):
                    self.trial_changed = True
                    self.task_changed = True
                    self.user_id_changed = True
                new_publish_status = config.publish_status_and_record
                
            if (new_publish_status != self.publish_status_and_record):
                self.trial_changed = True
                self.task_changed = True
                self.user_id_changed = True
            self.publish_status_and_record = new_publish_status

        config.publish_status_and_record = self.publish_status_and_record
        config.next_trial = False
        config.trial = str(self.trial)
        config.next_task = False
        config.task_id = self.task_id
        config.task_stop_status_files = False
        config.next_user = False
        config.user_id = self.format_user_id.format(UserID=self.user_id)
        
        self.callback_first_time = False;
        if not (self.led_client is None):
            self.led_client.send_goal(self.led_goal)

        return config
    

    def read_pid_status(self, status_file_name):
        file = open(self.external_status_files[status_file_name] + '.pid', 'r')
        pid = file.readline()
        file.close()
        return pid

    
    def write_manager_status_file(self):
        file = open(self.manager_status_file, 'w')
        file.writelines([self.study_name + "\n",
                         self.format_user_id.format(UserID=self.user_id) + "\n",
                         self.task_id + "\n",
                         str(self.trial) + "\n"])
        file.close()


    def write_status_files(self, status_files, command):
        if not (status_files is None):
            for status_file_name in status_files:
                rospy.logdebug("Writing {} to {}".format(command, status_file_name))
                file = open(self.external_status_files[status_file_name], 'w')
                file.writelines([command + "\n", 
                                 self.task_record_format.format(UserID=self.user_id, TaskID=self.task_id, TrialNr=self.trial)])
                file.close()


def main(args):
    user_study_manager = UserStudyManager()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
