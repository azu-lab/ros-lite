// [note] Auto-generated file
// [note] 2019-03-28T05:27:45Z
// [note] based on source/appl/ros_src/map/roslite_map_two_listenres.map



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
#if ROSLITE_TARGET_CLUSTER_ID == 0
    TIMapMutex.lock();
    TIMap.insert(std::make_pair(std::string("/chatter"), TopicInfo()));
    TIMap.at("/chatter").SIs.push_back(std::shared_ptr<SubscriberInfo>(new SubscriberInfoImpl<bool>("_chatter_2_0", NULL, 2, 0)));
    TIMap.at("/chatter").SIs.push_back(std::shared_ptr<SubscriberInfo>(new SubscriberInfoImpl<bool>("_chatter_2_1", NULL, 2, 0)));
    TIMapMutex.unlock();
#elif ROSLITE_TARGET_CLUSTER_ID == 1
    TIMapMutex.lock();
    TIMap.insert(std::make_pair(std::string("/chatter"), TopicInfo()));
    TIMap.at("/chatter").SIs.push_back(std::shared_ptr<SubscriberInfo>(new SubscriberInfoImpl<bool>("_chatter_2_0", NULL, 2, 0)));
    TIMap.at("/chatter").SIs.push_back(std::shared_ptr<SubscriberInfo>(new SubscriberInfoImpl<bool>("_chatter_2_1", NULL, 2, 0)));
    TIMap.at("/chatter").SIs.push_back(std::shared_ptr<SubscriberInfo>(new SubscriberInfoImpl<bool>("_chatter_0_0", NULL, 0, 0)));
    TIMapMutex.unlock();
#elif ROSLITE_TARGET_CLUSTER_ID == 2
    TIMapMutex.lock();
    TIMap.insert(std::make_pair(std::string("/chatter"), TopicInfo()));
    TIMap.at("/chatter").SIs.push_back(std::shared_ptr<SubscriberInfo>(new SubscriberInfoImpl<bool>("_chatter_0_0", NULL, 0, 0)));
    TIMapMutex.unlock();
#endif
}

}  // namespace ROSLITE_NAMESPACE
