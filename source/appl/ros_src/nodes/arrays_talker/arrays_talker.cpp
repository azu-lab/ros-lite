#include <ros/ros.h>
#include <test/Arrays.h>

#include <sstream>
#include <vector>

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "arrays_talker");

    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<test::Arrays>("/chatter", 1000);
    
    ros::Rate loop_rate(0.25);

    test::Arrays msg;
    msg.int8_arr.clear();
    int i = 0;

    while (ros::ok()) {
        ROS_INFO("published");
        
        msg.int8_arr.push_back(i);
        
        std::stringstream ss;
        ss << "hello world" << i;
        msg.string_arr.push_back(ss.str());
        
        ++i;

        chatter_pub.publish(msg);
        loop_rate.sleep();
    }
    
    return 0;
}