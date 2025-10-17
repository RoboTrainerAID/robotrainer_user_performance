#!/usr/bin/env python

import sys
import os
import roslib
roslib.load_manifest('diagnostic_updater')
import rospy
import rospkg
import fcntl  # Import the fcntl module for file locking

import actionlib
import diagnostic_updater
from iirob_led.msg import BlinkyAction, BlinkyGoal
from std_msgs.msg import String, ColorRGBA

import dynamic_reconfigure.server
from robotrainer_study_bayesian_optimization.cfg import UserStudyManagerConfig

from std_srvs.srv import Trigger, Empty
import yaml


class UserStudyManager:

    def __init__(self):
        rospy.init_node("user_study_manager")
        
        self.diagnostic = diagnostic_updater.Updater()
        self.diagnostic.add("User Study Manager", self.diagnostics_callback)
        self.diagnostic.setHardwareID("User_Study_Manager")
        self.diagnostic.broadcast(0, "Initializing User Study Manager")

        self.frequency = rospy.get_param("~frequency", 1)
        self.study_name = rospy.get_param("~study_name", "")
        self.user_id_prefix = rospy.get_param("~user_id_prefix", "")
        self.user_id_length = rospy.get_param("~user_id_length", 3)
        self.output_string_separator = rospy.get_param("~output_string_separator", "")
        self.use_led_output = rospy.get_param("~user_id_prefix", "")
        self.scenario_ns = "/" + rospy.get_param("/params/project_ns", "robotrainer") + "/" + rospy.get_param("/params/scenario_ns", "scenario")
        # self.scenario_folder = rospkg.RosPack().get_path("robotrainer_data_service") + "/yamls"
        self.scenario_folder = rospy.get_param("~scenario_folder_path")
        self.bag_folder = rospy.get_param("~bag_folder_path")
        self.initial_scenario = rospy.get_param("~initial_scenario")

        self.manager_status_file = rospy.get_param("~manager_status_file", None)
        self.data_set_from_file = False
        if (self.manager_status_file is None):
            self.manager_status_file = "/user_study_manager.status"
            rospy.logwarn("Study Manager uses default status file '{}'".format(self.manager_status_file))

        pkg_dir = rospkg.RosPack().get_path("robotrainer_study_bayesian_optimization")
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
                              self.output_string_separator + '{TaskID}' + self.output_string_separator + '{TrialNr}')
        self.task_record_format = self.format_string
        rospy.loginfo("The status string will have format: " + self.format_string)
        rospy.logwarn("Study status: " + self.format_string.format(
            UserID=self.user_id, TaskID=self.task_id, TrialNr=self.trial))

        # initialize publishers and action clients
        self.status_publisher_string = rospy.publisher = rospy.Publisher("~study_status", String, queue_size=1)
        self.rosbag_feedback_sub = rospy.Subscriber("/begin_write", String, self.rosbag_feedback_callback)
        # self.sync_srv = rospy.ServiceProxy("/rt2_sca_sync_node/send_sync_signal_1s", Trigger)
        self.deviation_reset = rospy.ServiceProxy("/robotrainer_deviation/reset", Trigger)
        self.deviation_configure = rospy.ServiceProxy("/robotrainer_deviation/configure", Trigger)
        self.led_client = actionlib.SimpleActionClient('/leds_rectangle/blinky', BlinkyAction)
        self.topic_check_srv = rospy.ServiceProxy("/topic_checker/start_check", Trigger)
        self.configure_modalities_srv = rospy.ServiceProxy("/base/configure_modalities", Empty)
        self.update_bo = rospy.ServiceProxy("/robotrainer_bayesian_optimization/update", Trigger)

        if self.led_client.wait_for_server(rospy.Duration(1)):
            self.led_goal = BlinkyGoal(ColorRGBA(0.8, 1.0, 0, 0.8), 10, 0.1, 0.1, 0, 0, 0, False, False)
            self.led_client.send_goal(self.led_goal)
        else:
            rospy.logerr("LED Server not found therefore it will not be used!")
            self.led_goal = None    
            self.led_client = None

        # check necessary topics for messages
        rospy.wait_for_service("/topic_checker/start_check", timeout=3)
        try:
            resp = self.topic_check_srv()
            if not resp.success:
                raise rospy.ServiceException(resp.message)
        except rospy.ServiceException as e:
            rospy.logerr("Topic check failed: {}".format(e))

        # Load initial scenario
        if self.initial_scenario:
            self.task_id = self.initial_scenario
            self.clear_scenario_params()
            self.load_scenario_params()

        # Initialize robotrainer_deviation service
        rospy.wait_for_service("/robotrainer_deviation/configure", timeout=3)
        try:
            resp = self.deviation_configure()
            if not resp.success:
                raise rospy.ServiceException(resp.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call robotrainer_deviation failed (Maybe scenario is not yet loaded, try again after clicking next user or next_task): {}".format(e))

        # initialize dynamic_reconfigure server
        self.first_reconfigure_callback = True
        self.dyn_cfg_srv = dynamic_reconfigure.server.Server(
          UserStudyManagerConfig, self.reconfigure_callback)

        # Timer for slower, potentially blocking logic
        rospy.Timer(rospy.Duration(1/self.frequency), self.logic_timer_callback)
        # New, high-frequency timer just for publishing the status string
        rospy.Timer(rospy.Duration(1/(2 * self.frequency)), self.publish_status_callback)

        self.diagnostic.force_update()

        rospy.loginfo("User Study Manager started")


    def publish_status_callback(self, event):
        """High-frequency, non-blocking callback to publish the current study status."""
        if self.user_id == -1 or self.trial == -1:
            # This check prevents publishing an invalid initial status
            return
        
        self.status_publisher_string.publish(
            self.format_string.format(
            UserID=self.user_id, TaskID=self.task_id, TrialNr=self.trial))


    def logic_timer_callback(self, event): 
        trigger_services = False
        self.diagnostic.update()

        if self.user_id == -1 or self.trial == -1:
            rospy.logerr_throttle(5, "User_id or Trial is not set therefore the study_status is not published!")
            return
            
        if self.trial_changed or self.task_changed or self.user_id_changed:
            self.write_study_manager_status_file()

            rospy.loginfo("New study status: \n" + "study_name: {}\nuser_id: {}\ntask_id: {}\ntrial: {}" \
                          .format(self.study_name, self.format_user_id.format(UserID=self.user_id), self.task_id, self.trial))
            
            trigger_services = True
            
        if self.task_changed:
            #TODO(Andreas) would be better if first, all scenario is cleared, then the robotrainer is brought back to its starting position, the operator triggers a checkbox and then the new scenario is loaded
            self.clear_scenario_params()

            if not self.task_id == "END":
                self.load_scenario_params()

            try:
                # Push the new scenario to the modalities with service /base/configure_modalities
                resp = self.configure_modalities_srv()
            except rospy.ServiceException as e:
                rospy.logerr("Configure modalities service failed: {}".format(e))

            try:
                resp = self.deviation_configure()
                if not resp.success:
                    raise rospy.ServiceException(resp.message)
            except rospy.ServiceException as e:
                rospy.logerr("deviation_configure service call failed: {}".format(e))

        if self.trial_changed:
            self.trial_changed = False

        if self.task_changed:
            self.task_changed = False
            
        if self.user_id_changed:
            self.user_id_changed = False

        if trigger_services:
            try:
                # Reset the deviation service
                resp = self.deviation_reset()
                if not resp.success:
                    raise rospy.ServiceException(resp.message)
            except rospy.ServiceException as e:
                rospy.logerr("deviation_reset service call failed: {}".format(e))

            try:
                # recheck if all necessary topics are still there   
                resp = self.topic_check_srv()
                if not resp.success:
                    raise rospy.ServiceException(resp.message)
            except rospy.ServiceException as e:
                rospy.logerr("topic_check service call failed: {}".format(e))

            # try:
            #     # Trigger the sync service (last)
            #     # rospy.logwarn("Waiting before triggering sync services")
            #     # rospy.sleep(rospy.Duration(2))
            #     resp = self.sync_srv()
            #     if not resp.success:
            #         raise rospy.ServiceException(resp.message)
            # except rospy.ServiceException as e:
            #     rospy.logerr("sync service call failed: {}".format(e))


    def clear_scenario_params(self):
        # First, clear the active scenario parameters by deleting the namespace on the parameter server.
        if rospy.has_param(self.scenario_ns):
            try:
                rospy.delete_param(self.scenario_ns)
                rospy.loginfo("Cleared scenario parameters in namespace: {}".format(self.scenario_ns))
            except Exception as e:
                rospy.logerr("Failed to clear parameters in namespace {}: {}".format(self.scenario_ns, e))
        else:
            rospy.logwarn("Namespace {} did not exist; nothing to clear.".format(self.scenario_ns))

    def load_scenario_params(self):
        scenario_file = os.path.join(self.scenario_folder, self.task_id + ".yaml")
        
        if os.path.isfile(scenario_file):
            with open(scenario_file, 'r') as f:
                params = yaml.safe_load(f)
            rospy.set_param(self.scenario_ns, params)
            rospy.loginfo("Loaded scenario parameters from {} into namespace {}".format(scenario_file, self.scenario_ns))
        else:
            rospy.logerr("Scenario file not found: {}".format(scenario_file))


    def rosbag_feedback_callback(self, message):
        # TODO(denis): Check if there is right thing started
        rospy.loginfo("\033[32mBag recording started: {}\033[0m".format(message.data))


    def diagnostics_callback(self, stat):
        if self.user_id == -1 or self.trial == -1:
            stat.summary(2, "UserID or Trial-Nr. is not set!")
        else:
            stat.summary(0, "Study status is published!")

        stat.add("Study name", self.study_name)
        stat.add("Current User", self.format_user_id.format(UserID=self.user_id))
        stat.add("Current Task", self.task_id)
        stat.add("Current Trial", self.trial)
                            
        return stat


    def reconfigure_callback(self, config, level):
        if not self.first_reconfigure_callback:

            next_trial = self.trial
            next_task = self.task_id
            next_user_id = self.user_id

            if (config.next_task):
                try:
                    resp = self.update_bo()
                    if not resp.success:
                        raise rospy.ServiceException(resp.message)
                    else:
                        rospy.loginfo("Update BO service call successful with new scenario: {}".format(resp.message))
                        next_task = resp.message                   
                except rospy.ServiceException as e:
                    rospy.logerr("Update BO service call failed: {}".format(e))
                if (self.trial == -1):
                    next_trial = 1
                else:
                    next_trial += 1

            if config.next_user:
                if (self.user_id == -1):
                    next_user_id = 1
                else:
                    next_user_id += 1
                next_trial = 1
                next_task = self.initial_scenario
            else:
                try:
                    user_id = int(config.user_id)
                    if (self.user_id != user_id):
                        if (user_id < (pow(10, self.user_id_length))):
                            next_user_id = user_id
                            next_trial = 1
                            next_task = self.initial_scenario
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
            
            if (config.start_topic_check):
                resp = self.topic_check_srv()
                if not resp.success:
                    rospy.logerr("Topic check failed: {}".format(resp.message))

           
            

        config.next_task = False
        config.next_user = False
        config.start_topic_check = False
        config.trial = str(self.trial)
        config.task_id = self.task_id
        config.user_id = self.format_user_id.format(UserID=self.user_id)
        
        self.first_reconfigure_callback = False
        if not (self.led_client is None):
            self.led_client.send_goal(self.led_goal)

        return config

    
    def write_study_manager_status_file(self):
        """Safely writes the current status to the file using an exclusive lock."""
        try:
            with open(self.manager_status_file, 'w') as f:
                # Acquire an exclusive lock. This will block other processes (readers/writers).
                fcntl.flock(f, fcntl.LOCK_EX)
                
                f.writelines([self.study_name + "\n",
                              self.format_user_id.format(UserID=self.user_id) + "\n",
                              self.task_id + "\n",
                              str(self.trial) + "\n"])
                
                # The lock is automatically released when the 'with' block exits.
        except IOError as e:
            rospy.logerr("Failed to write to manager status file {}: {}".format(self.manager_status_file, e))

    def load_study_manager_status_file(self):
        """Safely reads the status from the file using a shared lock."""
        try:
            # Open the file in 'a+' mode to create it if it doesn't exist, then prepare for reading.
            with open(self.manager_status_file, 'a+') as f:
                # Acquire a shared lock. This allows other readers but blocks writers.
                fcntl.flock(f, fcntl.LOCK_SH)
                
                # Go to the beginning of the file to read its contents
                f.seek(0)
                lines = f.readlines()

                # The lock is automatically released when the 'with' block exits.

            if len(lines) >= 4:
                self.study_name = lines[0].strip()
                self.user_id = int(lines[1].strip())
                self.task_id = lines[2].strip()
                self.trial = int(lines[3].strip())
                self.data_set_from_file = True
                rospy.loginfo("Manager status file successfully read and variables updated.")
            else:
                rospy.logwarn("Manager status file does not contain enough data. Using default parameters.")
        except (IOError, ValueError) as e:
            rospy.logwarn("Failed to read or parse manager status file: {}. Using default parameters.".format(e))


def main(args):
    user_study_manager = UserStudyManager()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
