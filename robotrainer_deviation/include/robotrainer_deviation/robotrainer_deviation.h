#ifndef ROBOTRAINER_USER_PERFORMANCE_DEVIATION_H
#define ROBOTRAINER_USER_PERFORMANCE_DEVIATION_H

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf2_ros/transform_listener.h>

#include <robotrainer_deviation/RobotrainerUserDeviation.h>
#include <robotrainer_deviation/PathIndex.h>
#include <robotrainer_parameters/shared_params.h>

#include <std_srvs/Trigger.h>

namespace robotrainer_user
{

class RobotrainerPathDeviation
{
public:
    RobotrainerPathDeviation(ros::NodeHandle &nh);

    bool configure(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

private:
    double getDistancePointLine(double px, double py, double l1x, double l1y, double l2x, double l2y);

    void update(const ros::TimerEvent &event);

    ros::NodeHandle nh_;

    // Objects and methods for locating the robot
    tf2_ros::Buffer *p_tf_buffer_;
    tf2_ros::TransformListener *p_tf_listener_;

    // Shared params
    robotrainer_parameters::SharedParams* sp_object_ptr_; ///< Object that manages the shared parameters
    robotrainer_parameters::SharedParamsParameters* ns_params_ptr_; ///< Commonly used parameters (namespace names)

    double calculateMarkerDeviation(std::string marker, int& current_path_index, visualization_msgs::Marker& visu_marker);

    bool updateMarkerLocation(std::string frame, geometry_msgs::TransformStamped& transform);

    // Vector-containers to hold the path tracking sections
    std::vector<tf2::Vector3> path_; ///< Stores the sections of the path, where pathtracking should be enabled. A path section consists of path points.

    ros::Publisher pub_deviation_;
    ros::Publisher pub_current_path_index_;
    ros::Publisher pub_deviation_markers_;

    std::string marker_front_link_;
    std::string marker_left_link_;
    std::string marker_right_link_;

    int current_path_index_front_;
    int current_path_index_left_;
    int current_path_index_right_;

    double update_rate_;

    ros::Timer updateTimer_;

    ros::ServiceServer srv_configure_;
};

};

#endif
