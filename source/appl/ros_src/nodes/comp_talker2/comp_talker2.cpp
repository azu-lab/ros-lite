#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "comp_talker2");

    ros::NodeHandle n;
    ros::Publisher chatter_pub1 = n.advertise<std_msgs::String>("/chatter2", 1000);
    ros::Publisher chatter_pub2 = n.advertise<std_msgs::String>("/chatter3", 1000);

    ros::Rate loop_rate(0.5);

    while (ros::ok()) {
        std_msgs::String msg;
        std::stringstream ss;

        ss << "hello";
        ROS_INFO("publish - %s", ss.str().c_str());
        msg.data = ss.str();
        chatter_pub1.publish(msg);
        chatter_pub2.publish(msg);
        loop_rate.sleep();
    }
    
    return 0;
}