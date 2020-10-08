#!/usr/bin/env python

import sys
import roslib
roslib.load_manifest('diagnostic_updater')
import rospy
import os
import subprocess

import actionlib
import diagnostic_updater
from iirob_led.msg import BlinkyAction, BlinkyGoal
from std_msgs.msg import String, ColorRGBA

import dynamic_reconfigure.server
from classical_user_performance.cfg import UserStudyManagerConfig


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
        # TODO: add here possibility to use custom list of user ids
        # self.user_id_list = rospy.get_param("~user_id_list", [None])
        # TODO: add here maximal number of trials for each task - could be nice used
        # self.num_trials = rospy.get_param("~num_trials", [None])
        
        ext_commands = rospy.get_param("~external_commands", None)
        self.external_commands = {}
        if not (ext_commands is None):
            for elem in ext_commands:
                self.external_commands.update(elem)
        self.external_commands_popen = {}
        self.run_command_on_task_change = rospy.get_param("~run_command_on_task_change", [None])
                                                          
        self.run_task_commands = True
        self.my_env = os.environ.copy()
        #self.my_env["PATH"] = "/usr/sbin:/sbin:" + self.my_env["PATH"]
        

        # initialize study variables
        if self.study_name:
          self.study_name += self.output_string_separator

        self.user_id = -1
        self.task_id = ""
        self.trial = -1
        self.publish_status = False

        self.format_user_id = '{UserID:0' + str(self.user_id_length) + 'd}'
        self.format_string = (self.study_name + self.user_id_prefix + self.format_user_id +
                              self.output_string_separator + '{TaskID}' + '-' + '{TrialNr}')
        rospy.loginfo("The status string will have format: " + self.format_string)

        # initialize publishers and action clients
        self.status_publisher_string = rospy.publisher = rospy.Publisher(
            "~study_status", String, queue_size=1)

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
        self.dyn_cfg_srv = dynamic_reconfigure.server.Server(
          UserStudyManagerConfig, self.reconfigure_callback)

        rospy.Timer(rospy.Duration(1/self.frequency), self.timer_callback)

        self.diagnostic.force_update()

    def timer_callback(self, event):
        if self.publish_status:
            if self.user_id != -1 and self.trial != -1:
                self.status_publisher_string.publish(
                  self.format_string.format(
                    UserID=self.user_id, TaskID=self.task_id, TrialNr=self.trial))
                  
                if self.run_task_commands:
                    self.run_commands(self.run_command_on_task_change)
                    self.run_task_commands = False
            else:
                rospy.logerr_throttle(5, "User_id or Trial is not set therefore the study_status is not published!")

        self.diagnostic.update()

    def diagnostics_callback(self, stat):
        if not self.publish_status:
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
        stat.add("Task run commands", self.run_task_commands);
        
        if not (self.external_commands is None):
            for key in self.external_commands.keys():
                status = "-----"
                if (key in self.external_commands_popen.keys()):
                    if not (self.external_commands_popen[key] is None):
                        self.external_commands_popen[key].poll()
                        if (self.external_commands_popen[key].returncode is None):
                            status = "Terminated"
                        else:
                            status = self.external_commands_popen[key].pid
                stat.add("Process {}".format(key), status)
            
        return stat

    def reconfigure_callback(self, config, level):
        # if called the first time set the parameters
        self.publish_status = config.publish_status

        if config.next_trial:
            if (self.trial == -1):
                self.trial = 1
            else:
                self.trial += 1
        else:
            try:
                if int(config.trial) >= 0:
                    self.trial = int(config.trial)
            except Exception as e:
                rospy.logerr(e)

        if config.next_task:
            next_id = (self.task_ids.index(self.task_id)+1) % len(self.task_ids)
            self.task_id = self.task_ids[next_id]
            self.trial = 1
            self.terminate_commands(self.run_command_on_task_change)
            self.run_task_commands = True
        elif config.task_id in self.task_ids:
            if (self.task_id != config.task_id):
                self.run_task_commands = True
            self.task_id = config.task_id
        else:
            rospy.logerr("TaskID: {TaskID} is not in the task list! \n \
                         Defined IDs are {TaskIDs}" \
                           .format(TaskID=config.task_id, TaskIDs=self.task_ids))

        if config.task_stop_commands:
            self.terminate_commands(self.run_command_on_task_change)

        if config.next_user:
            if (self.user_id == -1):
                self.user_id = 0
            else:
                self.user_id += 1
            self.trial = 1
            self.task_id = self.task_ids[0]
            self.publish_status = False
        else:
            try:
                user_id = int(config.user_id)
                if (user_id < (pow(10, self.user_id_length))):
                    self.user_id = user_id
                else:
                    rospy.logerr("UserID: {UserID} too large! \n \
                                Maximal UserID is {MaxUserIDs}" \
                                .format(UserID=config.task_id, MaxUserIDs=(pow(10, self.user_id_length) - 1)))

            except Exception as e:
                rospy.logerr(e)

        config.publish_status = self.publish_status
        config.next_trial = False
        config.trial = str(self.trial)
        config.next_task = False
        config.task_id = self.task_id
        config.task_stop_commands = False
        config.next_user = False
        config.user_id = self.format_user_id.format(UserID=self.user_id)

        #if not (self.led_client is None):
            #self.led_client.send_goal(self.led_goal)

        return config
    
    def run_commands(self, commands, command_sufix=""):
        if not (commands is None):
            for key in commands:
                if (key in self.external_commands_popen.keys() and \
                    not (self.external_commands_popen[key] is None)):
                    rospy.logerr("Process for the command is already running")
                else:
                    rospy.loginfo("Starting process {} with command {} {}".format(key, self.external_commands[key], command_sufix))
                    # see: 
                    # https://stackoverflow.com/questions/12060863/python-subprocess-call-a-bash-alias#25099813
                    # https://stackoverflow.com/questions/44418922/replicate-shell-environment-in-python-for-executing-bash-commands
                    self.external_commands_popen[key] = subprocess.Popen([self.external_commands[key], command_sufix], shell=True, executable='/usr/local/bin/interactive_bash', env=self.my_env)
                    
                    
    def terminate_commands(self, commands):
        if not (commands is None):
            for key in commands:
                if (key in self.external_commands_popen.keys() and \
                    not (self.external_commands_popen[key] is None)):
                    rospy.loginfo("Stopping process {} with command {}".format(key, commands[key]))
                    self.external_commands_popen[key].terminate()
                    self.external_commands_popen[key] = None
                else:
                    rospy.loginfo("Process for the command {} is not running. Nothing to stop.".format(key))


def main(args):
    user_study_manager = UserStudyManager()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
