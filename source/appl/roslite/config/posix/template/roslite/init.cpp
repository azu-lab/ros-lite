{% include 'common/note.cpp' %}


{% include 'common/generic_includes.cpp' %}


#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/init.h"
  #include "roslite/include/ros/info.h"
  #include "roslite/include/ros/debug.h"
  #include "roslite/include/std_msgs/String.h"
#else
  #include "ros/init.h"
  #include "ros/info.h"
  #include "ros/debug.h"
  #include "std_msgs/String.h"
#endif

#include <memory>
#include <vector>
#include <map>
#include <string>
#include <mutex>

namespace ROSLITE_NAMESPACE
{
    std::map<std::string, TopicInfo> TIMap;
    std::mutex TIMapMutex;

void generated_init(){
{% for cluster in cluster_list %}
    {% if loop.first %}
#if ROSLITE_TARGET_CLUSTER_ID == {{cluster}}
    {% else %}
#elif ROSLITE_TARGET_CLUSTER_ID == {{cluster}}
    {% endif %}
    TIMapMutex.lock();
    {% for topic in topic_list %}
    TIMap.insert(std::make_pair(std::string("{{topic}}"), TopicInfo()));
    {% endfor %}
    {% for info in subscriber_info_list_map[cluster] %}
    TIMap.at("{{info['topic']}}").SIs.push_back(std::shared_ptr<SubscriberInfo>(new SubscriberInfoImpl<bool>("{{info['name']}}", NULL, {{info['cluster']}}, 0)));
    {% endfor %}
    TIMapMutex.unlock();
    {% if loop.last %}
#endif
    {% endif %}
{% endfor %}
}

}  // namespace ROSLITE_NAMESPACE

