#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

void chatterCallback_t1(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s] in chatter6", msg->data.c_str());
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "comp_talker1");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/chatter6", 1000, chatterCallback_t1);
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("/chatter1", 1000);

    ros::Rate loop_rate(1);

    while (ros::ok()) {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "world";
        ROS_INFO("publish - %s", ss.str().c_str());
        msg.data = ss.str();
        
        chatter_pub.publish(msg);
        loop_rate.sleep();
    }
    
    return 0;
}