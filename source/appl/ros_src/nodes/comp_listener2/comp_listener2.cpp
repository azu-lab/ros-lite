#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

static ros::Publisher chatter_pub;
static std_msgs::String share_msg;

void chatterCallback_l2_1(const std_msgs::String::ConstPtr& msg) {
    // ROS_INFO("I heard: [%s] in comp_listener2", msg->data.c_str());
    share_msg.data = msg->data;
    chatter_pub.publish(share_msg);
}

void chatterCallback_l2_2(const std_msgs::String::ConstPtr& msg) {
    // ROS_INFO("I heard: [%s] in comp_listener2", msg->data.c_str());
    share_msg.data = msg->data;
    chatter_pub.publish(share_msg);
}

int
main(int argc, char **argv)
{   
    ros::init(argc, argv, "comp_listener2");

    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("/chatter1", 1000, chatterCallback_l2_1);
    ros::Subscriber sub2 = n.subscribe("/chatter2", 1000, chatterCallback_l2_2);
    chatter_pub = n.advertise<std_msgs::String>("/chatter5", 1000);

    return 0;
}