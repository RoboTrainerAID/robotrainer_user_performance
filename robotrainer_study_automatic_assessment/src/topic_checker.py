#!/usr/bin/env python
import yaml
import rospy
import rostopic
from threading import Event
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse
import time

class TopicChecker(object):
    def __init__(self):
        rospy.init_node("topic_checker")
        self.config_path = rospy.get_param("~topics_to_record_yaml")
        self.topics = self.load_topics_to_check(self.config_path)
        self.timeout = rospy.get_param("~topic_check_timeout", 5.0)
        self.service = rospy.Service("~start_check", Trigger, self.callback_start_check)

        self.topics_only_checked_by_subscribers = [
            "/base/virtual_forces/modalities_debug/position",
            "/base/virtual_forces/modalities_debug/velocity_in",
            "/base/virtual_forces/modalities_debug/velocity_out",
            "/base/virtual_forces/modalities_debug/resulting_velocity",
            "/base/virtual_forces/modalities_debug/resulting_force",
            "/base/virtual_forces/modalities_debug/status",
        ]
        # Remove topics that are checked by publisher only from the activity check list
        self.topics = [t for t in self.topics if t not in self.topics_only_checked_by_subscribers]

        rospy.loginfo("TopicChecker initialized. Service '~start_check' available.")

    def load_topics_to_check(self, yaml_path):
        try:
            with open(yaml_path, 'r') as f:
                config = yaml.safe_load(f)
        except Exception as e:
            rospy.logerr("Failed to read YAML file: %s", str(e))
            return []
        topics = config.get('topics_to_record_and_check_for_messages', [])
        if not isinstance(topics, list):
            rospy.logerr("Expected a list under 'topics_to_record_and_check_for_messages'")
            return []
        return [str(t) for t in topics if isinstance(t, str)]

    def check_topic_activity(self, topics, timeout=3.0):
        message_received = {t: Event() for t in topics}
        def make_callback(topic):
            def callback(msg):
                message_received[topic].set()
            return callback

        subscribers = []
        for topic in topics:
            try:
                topic_type, _, _ = rostopic.get_topic_class(topic)
                if topic_type is None:
                    rospy.logerr("Could not determine message type for topic: {}".format(topic))
                    continue
            except Exception as e:
                rospy.logerr("Exception resolving topic type for {}: {}".format(topic, e))
                continue
            sub = rospy.Subscriber(topic, topic_type, make_callback(topic), queue_size=1)
            subscribers.append(sub)

        start_time = time.time()
        elapsed = 0
        interval = 1.0
        last_report = 0

        while elapsed < timeout and not rospy.is_shutdown():
            time.sleep(0.1)
            elapsed = time.time() - start_time
            received_count = sum([event.is_set() for event in message_received.values()])
            if received_count == len(topics):
                # All topics received, exit early
                break
            if elapsed - last_report >= interval:
                rospy.logwarn("Checking topics for received messages: {}/{}".format(received_count, len(topics)))
                last_report = elapsed

        received_count = sum([event.is_set() for event in message_received.values()])
        all_received = received_count == len(topics)
        if all_received:
            rospy.loginfo("\033[32mAll topics received messages: {}/{}\033[0m".format(received_count, len(topics)))
            rospy.loginfo("\033[32mYou can now start the study and record topics!\033[0m")
        else:
            rospy.logerr("Some topics missing after {}s: {}/{}".format(timeout, received_count, len(topics)))
            for topic in topics:
                if not message_received[topic].is_set():
                    rospy.logerr("No messages received on topic '{}'".format(topic))
        for sub in subscribers:
            sub.unregister()
        del subscribers[:]
        del subscribers
        return all_received
    
    def check_topic_publishers(self):
        """
        Check if topics in self.topics_only_checked_by_subscribers have at least one publisher.
        Logs ros errors for missing publishers.
        """
        published_topics = rospy.get_published_topics()
        # Convert list of published topics (topic, type) into a simple list of topic names
        published_topic_names = [topic for topic, _ in published_topics]
        
        all_found = True
        for topic in self.topics_only_checked_by_subscribers:
            if topic not in published_topic_names:
                rospy.logerr("No publisher found for topic '{}'".format(topic))
                all_found = False
        return all_found
        

    def callback_start_check(self, req):
        res = TriggerResponse()
        if not self.topics:
            res.success = False
            res.message = "No topics to check. Check your YAML config."
            return res
        rospy.loginfo("Starting topic check by messages for {} topics and by publisher for {} topics...".format(len(self.topics), len(self.topics_only_checked_by_subscribers)))
        all_messages_ok = self.check_topic_activity(self.topics, self.timeout)
        all_publishers_ok = self.check_topic_publishers()
        if all_messages_ok and all_publishers_ok:
            res.success = True
            res.message = "All topics received messages."
        else:
            res.success = False
            res.message = "Some topics did not receive messages."
        return res

def main():
    checker = TopicChecker()
    rospy.spin()

if __name__ == "__main__":
    main()