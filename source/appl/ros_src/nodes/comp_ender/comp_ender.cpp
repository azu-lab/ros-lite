#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

void chatterCallback_e_1(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s] in chatter4", msg->data.c_str());
}

void chatterCallback_e_2(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s] in chatter5", msg->data.c_str());
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "comp_ender");

    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("/chatter4", 1000, chatterCallback_e_1);
    ros::Subscriber sub2 = n.subscribe("/chatter5", 1000, chatterCallback_e_2);
    
    return 0;
}