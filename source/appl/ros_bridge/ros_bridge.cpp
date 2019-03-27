#include <ros/ros.h>
#include <roslite/include/ros/ros.h>
#include "ros_bridge.h"

int rosl_bridge_generated_main(ros::NodeHandle ros_nh, roslite::NodeHandle roslite_nh);

void ros_bridge_main()
{
    char *argv = (char *)"ros_bridge";
    int argc = 1;
    
    ros::init(argc, &argv, ROS_BRIDGE_NODE_NAME);
    roslite::init(argc, &argv, "/ros_bridge");
    
    ros::NodeHandle ros_nh;
    roslite::NodeHandle roslite_nh;

    rosl_bridge_generated_main(ros_nh, roslite_nh);

    ros::shutdown();

    exit(0);
}
