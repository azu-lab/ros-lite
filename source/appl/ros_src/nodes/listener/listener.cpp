#include <ros/ros.h>
#include <std_msgs/String.h>

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
    // ROS_INFO("[cb] msg: %s", msg->data.c_str());
    // ROS_INFO("[cb] size: %d", msg->data.length());
}

int
main(int argc, char **argv)
{

    ros::init(argc, argv, "listener");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/chatter", 1000, chatterCallback);

    ros::spin();
    return 0;
}
