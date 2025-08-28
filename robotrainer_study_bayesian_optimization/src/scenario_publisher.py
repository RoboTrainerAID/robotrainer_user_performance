#!/usr/bin/env python

import rospy
import os
import yaml
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class ScenarioPublisher:
    def __init__(self):

        # topic_name = rospy.get_param('/gait_estimation/topic_name', True)
        self.study_status = rospy.Subscriber("/robotrainer_user_study_manager/study_status", String, self.listen_study_status)
        self.scenario_folder = rospy.get_param('/scenario_publisher/scenario_folder', '/robotrainer_user_performance/scenarios')
        self.newton_per_meter = rospy.get_param('/scenario_publisher/newton_per_meter', 30.0)
        self.publish_frequency = rospy.get_param('/scenario_publisher/publish_frequency', 1.0)  # Hz
        self.last_published_time = rospy.Time.now()
        self.last_scenario_name = None
        self.scenario = {}

        # Publishers for visualization markers
        self.path_pub = rospy.Publisher("/scenario_publisher/visualization_marker", Marker, queue_size=10)
        self.force_pub = rospy.Publisher("/scenario_publisher/visualization_marker", Marker, queue_size=10)

        rospy.loginfo("Start scenario_publisher from scenario folder: " + self.scenario_folder)


    def listen_study_status(self, data):
        # data: "KATE_AA_U010_16_yellow_line_force_right_60_1"

        parts = data.data.split('_')
        study_name = parts[0] + '_' + parts[1]  # "KATE_AA"
        user_id = int(parts[2][1:])  # "010" (remove leading 'U')
        trial = parts[-1]  # Last part is the trial
        scenario_name = '_'.join(parts[3:-1])  # Everything between becomes the scenario name
        
        # rospy.loginfo("study_name=%s, user_id=%d, scenario=%s, trial=%s" % (study_name, user_id, scenario_name, trial))

        if self.last_scenario_name != scenario_name:
            self.load_scenario_params(scenario_name)
            
            # force direct publish on scenario change
            self.last_published_time = rospy.Time.now() - rospy.Duration(1.0 / self.publish_frequency)

        if not self.scenario:
            return
        
        current_time = rospy.Time.now()
        if (current_time - self.last_published_time).to_sec() < 1.0 / self.publish_frequency:
            return  # Skip publishing if not enough time has passed
        self.last_published_time = current_time
        
        # Publish path marker
        self.publish_path(self.scenario.get('path', {}))

        # Publish force markers for each force name in the scenario
        for force in self.scenario.get('force', {}).get('config', {}).get('force_names', []):
            self.publish_force(force)
        
    def publish_path(self, path_data):
        """
        Creates and publishes a LINE_STRIP marker using the point data
        from the scenario file. It uses the "points" field for ordering if available.
        """
        if not isinstance(path_data, dict):
            rospy.logerr("Invalid path data")
            return

        # Determine point order.
        if 'points' in path_data and isinstance(path_data['points'], list):
            points_order = path_data['points']
        else:
            # Fallback: use sorted keys starting with 'point'
            points_order = sorted([k for k in path_data.keys() if k.startswith('point')])

        if not points_order:
            rospy.logwarn("No points found in path data")
            return

        marker = Marker()
        marker.header.frame_id = "map"  # Adjust frame if needed
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Line width
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.points = []
        for pt_name in points_order:
            pt_data = path_data.get(pt_name, None)
            if pt_data and isinstance(pt_data, dict):
                p = Point()
                p.x = pt_data.get('x', 0)
                p.y = pt_data.get('y', 0)
                p.z = pt_data.get('z', 0)
                marker.points.append(p)
        self.path_pub.publish(marker)

    def publish_force(self, force_name):
        """
        Creates and publishes an ARROW marker, a CYLINDER marker (circle), and a TEXT marker for the given force.
        The arrow is drawn from the "area" along the "arrow" vector, with the arrow's head length scaled down by the newton_per_meter.
        The circle is drawn at the "area" position with a radius computed from the distance between "area" and the "margin" position.
        A text marker is displayed at the end of the arrow showing the force strength in Newtons.
        """
        force_section = self.scenario.get('force', {})
        force_data = force_section.get('data', {}).get(force_name, None)
        if not force_data:
            rospy.logwarn("No force data for " + force_name)
            return

        # Retrieve points from force data
        area = force_data.get('area', {})
        arrow = force_data.get('arrow', {})
        margin = force_data.get('margin', {})  # Use 'margin' key for circle radius

        start = Point()
        start.x = area.get('x', 0)
        start.y = area.get('y', 0)
        start.z = area.get('z', 0)

        arrow_vector = Point()
        arrow_vector.x = arrow.get('x', 0) / self.newton_per_meter  # Scale down by newton_per_meter
        arrow_vector.y = arrow.get('y', 0) / self.newton_per_meter
        arrow_vector.z = arrow.get('z', 0) / self.newton_per_meter

        end = Point()
        end.z = start.z  # Keep z coordinate the same
        end.x = start.x + arrow_vector.x
        end.y = start.y + arrow_vector.y

        length = (arrow_vector.x*arrow_vector.x + arrow_vector.y*arrow_vector.y)**0.5

        # Compute force strength in Newtons: arrow length scaled by newton_per_meter
        force_strength = length * self.newton_per_meter
        rospy.loginfo("Force [%s]: strength = %.2f N", force_name, force_strength)

        # Create arrow marker (visual representation of force)
        arrow_marker = Marker()
        arrow_marker.header.frame_id = "map"
        arrow_marker.header.stamp = rospy.Time.now()
        arrow_marker.ns = "force_arrow"
        arrow_marker.id = abs(hash(force_name)) % 10000  # Unique id for arrow marker
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        arrow_marker.points = [start, end]
        arrow_marker.pose.orientation.x = 0.0
        arrow_marker.pose.orientation.y = 0.0
        arrow_marker.pose.orientation.z = 0.0
        arrow_marker.pose.orientation.w = 1.0
        arrow_marker.scale.x = 0.1  # Shaft diameter
        arrow_marker.scale.y = 0.2  # Head diameter
        arrow_marker.scale.z = 0.3 * length
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 0.2
        arrow_marker.color.b = 1.0
        arrow_marker.color.a = 1.0
        self.force_pub.publish(arrow_marker)

        # Get margin position
        margin_pos = Point()
        margin_pos.x = margin.get('x', 0)
        margin_pos.y = margin.get('y', 0)
        margin_pos.z = margin.get('z', 0)

        # Compute radius as the distance (in x,y-plane) from the area to the margin position.
        rad = ((start.x - margin_pos.x)**2 + (start.y - margin_pos.y)**2)**0.5

        # Create circle marker as a thin cylinder representing the force area.
        circle_marker = Marker()
        circle_marker.header.frame_id = "map"
        circle_marker.header.stamp = rospy.Time.now()
        circle_marker.ns = "force_circle"
        circle_marker.id = (abs(hash(force_name)) % 10000) + 10000 # Unique id
        circle_marker.type = Marker.CYLINDER
        circle_marker.action = Marker.ADD
        circle_marker.pose.position = start
        circle_marker.pose.orientation.x = 0.0
        circle_marker.pose.orientation.y = 0.0
        circle_marker.pose.orientation.z = 0.0
        circle_marker.pose.orientation.w = 1.0
        circle_marker.scale.x = rad * 2  # Diameter in x
        circle_marker.scale.y = rad * 2  # Diameter in y
        circle_marker.scale.z = 0.001    # Very thin height for a circle
        circle_marker.color.r = 0.0
        circle_marker.color.g = 0.2
        circle_marker.color.b = 1.0
        circle_marker.color.a = 0.3
        self.force_pub.publish(circle_marker)

        # Create text marker at the end of the arrow to display force strength
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "force_text"
        text_marker.id = (abs(hash(force_name)) % 10000) + 20000 # Unique id
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.scale.z = 0.2  # Set scale for the text
        text_marker.pose.position.x = end.x
        text_marker.pose.position.y = end.y
        text_marker.pose.position.z = end.z
        text_marker.pose.orientation.x = 0.0
        text_marker.pose.orientation.y = 0.0
        text_marker.pose.orientation.z = 0.0
        text_marker.pose.orientation.w = 1.0
        text_marker.color.r = 0.0
        text_marker.color.g = 0.2
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = "{:.2f}N".format(force_strength)
        self.force_pub.publish(text_marker)
        
    def load_scenario_params(self, scenario_name):
        scenario_file = os.path.join(self.scenario_folder, scenario_name + ".yaml")
        
        if os.path.isfile(scenario_file):
            with open(scenario_file, 'r') as f:
                self.scenario = yaml.safe_load(f)
            # rospy.set_param(self.scenario_ns, self.scenario)
            self.last_scenario_name = scenario_name
            rospy.loginfo("Loaded scenario parameters from {}".format(scenario_file))
        else:
            rospy.logerr("Scenario file not found: {}".format(scenario_file))


if __name__ == '__main__':
    rospy.init_node('scenario_publisher', anonymous=True)
    rospy.get_rostime()
    rospy.get_time()

    node = ScenarioPublisher()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass