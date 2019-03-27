#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

static ros::Publisher chatter_pub;

void chatterCallback_l1(const std_msgs::String::ConstPtr& msg) {
    // ROS_INFO("I heard: [%s] in comp_listener1", msg->data.c_str());
    std_msgs::String send_msg;
    std::stringstream ss;
    ss << "checked " << msg->data;
    send_msg.data = ss.str();
    chatter_pub.publish(send_msg);
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "comp_listener1");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/chatter1", 1000, chatterCallback_l1);
    chatter_pub = n.advertise<std_msgs::String>("/chatter4", 1000);    
    
    return 0;
}