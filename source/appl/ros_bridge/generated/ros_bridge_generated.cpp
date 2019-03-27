// [note] Auto-generated file
// [note] 2019-03-20T08:43:51Z
// [note] based on source/appl/ros_src/map/roslite_map_two_listenres.map

#include <ros/ros.h>
#include <roslite/include/ros/ros.h>
#include "../ros_bridge.h"

#include <std_msgs/String.h>
#include <roslite/include/std_msgs/String.h>
ros::Publisher ros_pub__chatter;
roslite::Subscriber roslite_sub__chatter;
void roslite__chatter_Callback(const roslite_std_msgs::String::ConstPtr& roslite_msg) {
    ros_pub__chatter.publish(roslite_msg->ToRosMsgPtr());
}


#include <std_msgs/String.h>
#include <roslite/include/std_msgs/String.h>
roslite::Publisher roslite_pub__chatter;
ros::Subscriber ros_sub__chatter;
void ros__chatter_Callback(const ros::MessageEvent<std_msgs::String const>& ros_msg) {
    if (ros_msg.getPublisherName() == ROS_BRIDGE_NODE_NAME_FULL_PATH) {
        // Ignore message from myself.
        return;
    }
    roslite_pub__chatter.publish(*roslite_std_msgs::String::FromRosMsgPtr(ros_msg.getMessage()));
}


int rosl_bridge_generated_main(ros::NodeHandle ros_nh, roslite::NodeHandle roslite_nh)
{
    roslite_sub__chatter = roslite_nh.subscribe("/chatter", 10, roslite__chatter_Callback);
    ros_pub__chatter = ros_nh.advertise<std_msgs::String>("/chatter", 10);
    
    ros_sub__chatter = ros_nh.subscribe("/chatter", 10, ros__chatter_Callback);
    roslite_pub__chatter = roslite_nh.advertise<roslite_std_msgs::String>("/chatter", 10);
    
    while (ros::ok() && roslite::ok()) {
        ros::spinOnce();
        roslite::spinOnce();
        roslite_thread_delay(1);
    }

    ros_pub__chatter.shutdown();

    ros_sub__chatter.shutdown();
}