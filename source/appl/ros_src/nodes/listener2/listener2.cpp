#include <ros/ros.h>
#include <std_msgs/String.h>

void chatterCallback2(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s] in listener2", msg->data.c_str());
}

int
main(int argc, char **argv)
{

    ros::init(argc, argv, "listener2");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/chatter", 1000, chatterCallback2);

    ros::spin();
    return 0;
}
