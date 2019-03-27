#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/info.h"
  #include "roslite/include/ros/serialization.h"
#else
  #include "ros/info.h"
  #include "ros/serialization.h"
#endif

#include <string>
#include <vector>
#include <memory>

namespace ROSLITE_NAMESPACE
{

SubscriberInfo::SubscriberInfo(const std::string& recv_tname, const uint8_t assigned_hwclid, roslite_id_t main_tid)
: recv_tname_(recv_tname)
, assigned_hwclid_(assigned_hwclid)
, main_tid_(main_tid)
{}

roslite_id_t SubscriberInfo::getTid(){
    // roslite_debug_printf("[SubscriberInfoImpl] recv_tname_: %s\n", recv_tname_.c_str());
    return (roslite_id_t)roslite_name_lookup(recv_tname_.c_str(), ROSLITE_NSID_GLOBAL);
}

roslite_id_t SubscriberInfo::getMainTid(){
    return main_tid_;
}

TopicInfo::TopicInfo()
{}

uint32_t TopicInfo::get_cluster_index(){
    // roslite_debug_printf("[subscribe] cluster_index: %d\n", cluster_index);
    return cluster_index_++;
}

}  // namespace ROSLITE_NAMESPACE
