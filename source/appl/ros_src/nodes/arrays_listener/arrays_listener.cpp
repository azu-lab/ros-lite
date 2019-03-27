#include <ros/ros.h>
#include <test/Arrays.h>

#include <vector>

void chatterCallback(const test::Arrays::ConstPtr& msg) {
    for(auto it = msg->int8_arr.begin(); it != msg->int8_arr.end(); ++it){
        ROS_INFO("I heard: int8_arr [%d]", *it);
    }
    for(auto it = msg->string_arr.begin(); it != msg->string_arr.end(); ++it){
        ROS_INFO("I heard: string_arr [%s]", it->c_str());
    }
    ROS_INFO("--");
}

int
main(int argc, char **argv)
{

    ros::init(argc, argv, "arrays_listener");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/chatter", 1000, chatterCallback);

    ros::spin();
    return 0;
}
