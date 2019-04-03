#include <robotrainer_deviation/robotrainer_deviation.h>

namespace robotrainer_user
{

RobotrainerPathDeviation::RobotrainerPathDeviation(ros::NodeHandle& nh): nh_(nh)
{
    // Initialize tf2 objects
    p_tf_buffer_ = new tf2_ros::Buffer();
    p_tf_listener_ = new tf2_ros::TransformListener(*p_tf_buffer_, true);

    // Get common params
    sp_object_ptr_ = new robotrainer_parameters::SharedParams();
    ns_params_ptr_ = sp_object_ptr_->getParamsPtr();

    // Check if shared params exist with a default value.
    if(ns_params_ptr_->project_ns == ""
        || ns_params_ptr_->scenario_ns == ""
        || ns_params_ptr_->force_ns == ""
        || ns_params_ptr_->config_ns == ""
        || ns_params_ptr_->data_ns == ""
        || ns_params_ptr_->arrow_ns == ""
        || ns_params_ptr_->area_ns == ""
        || ns_params_ptr_->margin_ns == ""
        || ns_params_ptr_->force_distance_function_ns == "")
    {
        ROS_ERROR("RobotrainerDeviation: Namespaces are not configured properly.");
    }

    nh_.param<std::string>("marker_front_link", marker_front_link_, "robotrainer_front_marker");
    nh_.param<std::string>("marker_left_link", marker_left_link_, "robotrainer_left_marker");
    nh_.param<std::string>("marker_right_link", marker_right_link_, "robotrainer_right_marker");
    nh_.param<double>("update_rate", update_rate_, 20);

    pub_deviation_ = nh_.advertise<robotrainer_deviation::RobotrainerUserDeviation>("robotrainer_deviation", 10);
    pub_deviation_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("robotrainer_deviation_markers", 10);

    srv_configure_ = nh_.advertiseService("Configure", &RobotrainerPathDeviation::configure, this);

    updateTimer_ = nh_.createTimer(ros::Rate(update_rate_), &RobotrainerPathDeviation::update, this, false, false);
    //TODO: add configure service
}

void RobotrainerPathDeviation::update(const ros::TimerEvent &event)
{
    // If there is no path defined, return.
    if (path_.size() < 1)
    {
        return;
    }

    // Calculate distance to center
    visualization_msgs::MarkerArray visu_markers;
    robotrainer_deviation::RobotrainerUserDeviation deviation;

    // Calculate front
    visualization_msgs::Marker visu_marker_front;
    visu_marker_front.id = 1;
    visu_marker_front.color.a = 1.0;
    visu_marker_front.color.r = 0.0;
    visu_marker_front.color.g = 1.0;
    visu_marker_front.color.b = 0.0;
    deviation.front = calculateMarkerDeviation(marker_front_link_, current_path_index_front_, visu_marker_front);
    visu_markers.markers.push_back(visu_marker_front);

    // Calculate left
    visualization_msgs::Marker visu_marker_left;
    visu_marker_left.id = 2;
    visu_marker_left.color.a = 1.0;
    visu_marker_left.color.r = 0.0;
    visu_marker_left.color.g = 0.0;
    visu_marker_left.color.b = 1.0;
    deviation.left = calculateMarkerDeviation(marker_left_link_, current_path_index_left_, visu_marker_left);
    visu_markers.markers.push_back(visu_marker_left);

    // Calculate right
    visualization_msgs::Marker visu_marker_right;
    visu_marker_right.id = 3;
    visu_marker_right.color.a = 1.0;
    visu_marker_right.color.r = 1.0;
    visu_marker_right.color.g = 0.0;
    visu_marker_right.color.b = 0.0;
    deviation.right = calculateMarkerDeviation(marker_right_link_, current_path_index_right_, visu_marker_right);
    visu_markers.markers.push_back(visu_marker_right);
}

double RobotrainerPathDeviation::getDistancePointLine(double px, double py, double l1x, double l1y, double l2x, double l2y)
{
    return std::abs((l2y - l1y) * px - (l2x - l1x) * py + l2x * l1y - l2y * l1x) / sqrt(pow(l2y - l1y, 2) + pow(l2x - l1x, 2));
}

double RobotrainerPathDeviation::calculateMarkerDeviation(std::string marker, int& current_path_index, visualization_msgs::Marker& visu_marker)
{
    double deviation = -1;

    geometry_msgs::TransformStamped marker_transform;
    if (!updateMarkerLocation(marker, marker_transform))
    {
        return deviation;
    }
    tf2::Vector3 marker_location;
    tf2::fromMsg(marker_transform.transform.translation, marker_location);

    int direction = 1;;

    if (current_path_index == path_.size())
    {
        current_path_index = -1;
        return deviation;
    }


    if (current_path_index == -1)
    {
        double dist_front = tf2::tf2Distance(path_.front(), marker_location);
//         double dist_back = tf2::tf2Distance(path_.back(), marker_location);

         if (dist_front < 0.3)
        {
            current_path_index = 0;
        }
// TODO: If needed for back/front
//         double dist_back = tf2::tf2Distance(path_.back(), marker_location);
//         if (dist_front < 0.3 and dist_back < 0.3) {
//             if (dist_front < dist_back) {
//
//             }
//
//         else {
//             dist = tf2::tf2Distance(path_.back(), marker_location);
//
        else {
            return deviation;
        }
    }
    double line_next_distance, line_previous_distance = -1;
    if (current_path_index+1 < path_.size()) {
        line_next_distance = getDistancePointLine(marker_location.x(), marker_location.y(), path_[current_path_index].x(), path_[current_path_index].y(), path_[current_path_index+1].x(), path_[current_path_index+1].y());
    }
    if (current_path_index > 0) {
        line_previous_distance = getDistancePointLine(marker_location.x(), marker_location.y(), path_[current_path_index-1].x(), path_[current_path_index-1].y(), path_[current_path_index].x(), path_[current_path_index].y());
    }

    if (line_previous_distance != -1 and line_next_distance != -1 and (line_previous_distance < line_next_distance)) {
        direction = -1;
    }

    if (tf2::tf2Distance(path_[current_path_index], marker_location) - tf2::tf2Distance(path_[current_path_index+direction], marker_location) > 0.05)
    {
        current_path_index += direction;
    }

    visu_marker.header.frame_id = "map";
    visu_marker.header.stamp = ros::Time();
    visu_marker.ns = "robotrainer_deviation";
    visu_marker.type = visualization_msgs::Marker::ARROW;
    visu_marker.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point start_point, end_point;
    tf2::toMsg(marker_location, start_point);
    visu_marker.points.clear();
    visu_marker.points.push_back(start_point);
    tf2::toMsg(path_[current_path_index], end_point);
    visu_marker.points.push_back(end_point);
    visu_marker.scale.x = 0.01;
    visu_marker.scale.y = 0.015;

    return deviation;

}

bool RobotrainerPathDeviation::updateMarkerLocation(std::string frame, geometry_msgs::TransformStamped& transform)
{
    bool ret = false;
    try
    {
        transform = p_tf_buffer_->lookupTransform("map", frame, ros::Time(0));
        ret = true;
    }
    catch(tf2::TransformException &ex)
    {
        ROS_ERROR_THROTTLE(1, "%s", ex.what());
    }
    return ret;
}

bool RobotrainerPathDeviation::configure(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

    updateTimer_.stop();

    path_.clear();

    // Namespaces
    std::string path_path = "/" + ns_params_ptr_->project_ns + "/" + ns_params_ptr_->scenario_ns + "/" + ns_params_ptr_->path_ns;

    // Retrieve path point names
    std::vector<std::string> point_names;
    ros::param::get(path_path + "/points", point_names);
    if (point_names.size() < 2)
    {
        ROS_WARN_STREAM("RobotrainerDeviation: No valid path configuration found in namespace " << path_path << ".\n RobotrainerDeviation-will not be used!");
        return true;
    }
    ROS_INFO_STREAM("RobotrainerDeviation: Found a path configuration in namespace " << path_path << " with " << point_names.size() << " points.");

    for(int i = 0; i < point_names.size(); i++)
    {
            double x, y, z;
            ros::param::get(path_path + "/" + point_names[i] + "/x", x);
            ros::param::get(path_path + "/" + point_names[i] + "/y", y);
            ros::param::get(path_path + "/" + point_names[i] + "/z", z);
            tf2::Vector3 point(x,y,z);
            path_.push_back(point);
    }

    // Initialize current path indexes
    current_path_index_front_ = -1;
    current_path_index_left_ = -1;
    current_path_index_right_ = -1;

    updateTimer_.start();

    return true;
}

};

//// Some copy


//    // Update the current robot location stored in transform_map_robot_
//     updateRobotLocation();
//     tf2::Vector3 robot_location;
//     tf2::fromMsg(transform_map_robot_.transform.translation, robot_location);
// //     pub_position_.publish(tf2::toMsg(robot_location)); // debug
//
//     int direction = 1;;
//
//     if (current_path_index_ == -1)
//     {
//         double dist_front = tf2::tf2Distance(path_.front(), robot_location);
//
//          if (dist_front < 0.3)
//         {
//             current_path_index_ = 0;
//         }
// // TODO: If needed for back/front
// //         double dist_back = tf2::tf2Distance(path_.back(), robot_location);
// //         if (dist_front < 0.3 and dist_back < 0.3) {
// //             if (dist_front < dist_back) {
// //
// //             }
// //
// //         else {
// //             dist = tf2::tf2Distance(path_.back(), robot_location);
// //
//         else {
//             return;
//         }
//     }
//
//     geometry_msgs::PointStamped active_point;
//
//     active_point.header.frame_id = 'map';
//     active_point.point = path_[current_path_index_];
// //     active_point.point.x = path_[current_path_index_].x;
// //     active_point.point.y = path_[current_path_index_].y;
// //     active_point.point.z = ;
//
//     tf2::Vector3 next_point = path_[current_path_index_+1];
//     tf2::Vector3 line_next = next_path_point - path_[current_path_index_];
//     line_next.normalize();
//
//     tf2::Vector3 next_halfplane(-line.y(), line.x(), -(-line_next.y()*next_point.x() + line_next.x()*next_point.y()));
//
//     double line_next_distance = next_halfplane.x()*robot_location.x() + next_halfplane.y()*robot_location.y();
//
//     if (current_path_index_ > 0) {
//         tf2::Vector3 previous_point = path_[current_path_index_-1];
//         tf2::Vector3 line_previous = next_path_point - path_[current_path_index_];
//         line_previous.normalize();
//
//         tf2::Vector3 previous_halfplane(-line.y(), line.x(), -(-line_previous.y()*next_point.x() + line_previous.x()*next_point.y()));
//
//         double line_previous_distance = previous_halfplane.x()*robot_location.x() + previous_halfplane.y()*robot_location.y();
//
//         if (line_previous_distance < line_next_distance) {
//             direction = -1;
//         }
//     }
//
//     if (tf2::tf2Distance(path_[current_path_index_], robot_location) - tf2::tf2Distance(path_[current_path_index_+direction], robot_location) < 0.05)
//     {
//         current_path_index_ += direction;
//     }
