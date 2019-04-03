#include <robotrainer_deviation/robotrainer_deviation.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotrainer_deviation");

    ros::NodeHandle nh("~");

    new robotrainer_user::RobotrainerPathDeviation(nh);

    ROS_INFO("RobotrainerPathDeviation Node running.");

    ros::waitForShutdown();

    return 0;
}

