#pragma once

#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/common.h"
  #include "roslite/include/ros/info.h"
#else
  #include "ros/common.h"
  #include "ros/info.h"
#endif

#include <map>
#include <string>
#include <mutex>

namespace ROSLITE_NAMESPACE
{
extern std::map<std::string, TopicInfo> TIMap;
extern std::mutex TIMapMutex;
}
