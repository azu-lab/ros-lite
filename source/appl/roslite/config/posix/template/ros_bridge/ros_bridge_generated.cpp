{% include 'common/note.cpp' %}


#include <ros/ros.h>
#include <roslite/include/ros/ros.h>
#include "../ros_bridge.h"

{% for topic in pub_topic_infos %}
#include <{{topic['type_package']}}/{{topic['type_name']}}.h>
#include <roslite/include/{{topic['type_package']}}/{{topic['type_name']}}.h>
ros::Publisher ros_pub_{{topic['topic_no_slash']}};
roslite::Subscriber roslite_sub_{{topic['topic_no_slash']}};
void roslite_{{topic['topic_no_slash']}}_Callback(const roslite_{{topic['type']}}::ConstPtr& roslite_msg) {
    ros_pub_{{topic['topic_no_slash']}}.publish(roslite_msg->ToRosMsgPtr());
}

{% endfor %}

{% for topic in sub_topic_infos %}
#include <{{topic['type_package']}}/{{topic['type_name']}}.h>
#include <roslite/include/{{topic['type_package']}}/{{topic['type_name']}}.h>
roslite::Publisher roslite_pub_{{topic['topic_no_slash']}};
ros::Subscriber ros_sub_{{topic['topic_no_slash']}};
void ros_{{topic['topic_no_slash']}}_Callback(const ros::MessageEvent<{{topic['type']}} const>& ros_msg) {
    if (ros_msg.getPublisherName() == ROS_BRIDGE_NODE_NAME_FULL_PATH) {
        // Ignore message from myself.
        return;
    }
    roslite_pub_{{topic['topic_no_slash']}}.publish(*roslite_{{topic['type']}}::FromRosMsgPtr(ros_msg.getMessage()));
}

{% endfor %}

int rosl_bridge_generated_main(ros::NodeHandle ros_nh, roslite::NodeHandle roslite_nh)
{
    {% for topic in pub_topic_infos %}
    roslite_sub_{{topic['topic_no_slash']}} = roslite_nh.subscribe("{{topic['topic']}}", 10, roslite_{{topic['topic_no_slash']}}_Callback);
    ros_pub_{{topic['topic_no_slash']}} = ros_nh.advertise<{{topic['type']}}>("{{topic['topic']}}", 10);
    {% endfor %}
    
    {% for topic in sub_topic_infos %}
    ros_sub_{{topic['topic_no_slash']}} = ros_nh.subscribe("{{topic['topic']}}", 10, ros_{{topic['topic_no_slash']}}_Callback);
    roslite_pub_{{topic['topic_no_slash']}} = roslite_nh.advertise<roslite_{{topic['type']}}>("{{topic['topic']}}", 10);
    {% endfor %}
    
    while (ros::ok() && roslite::ok()) {
        ros::spinOnce();
        roslite::spinOnce();
        roslite_thread_delay(1);
    }

    {% for topic in pub_topic_infos %}
    ros_pub_{{topic['topic_no_slash']}}.shutdown();
    {% endfor %}

    {% for topic in sub_topic_infos %}
    ros_sub_{{topic['topic_no_slash']}}.shutdown();
    {% endfor %}
}
