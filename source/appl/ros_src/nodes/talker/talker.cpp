#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("/chatter", 1000);
    
    ros::Rate loop_rate(1);

    while (ros::ok()) {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world";
        ROS_INFO("%s", ss.str().c_str());
        msg.data = ss.str();
        // ROS_INFO("msg: %s", msg.data.c_str());
        // ROS_INFO("size: %d", msg.data.length());
        
        chatter_pub.publish(msg);
        loop_rate.sleep();
    }
    
    return 0;
}
