#pragma once

#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/common.h"
#else
  #include "ros/common.h"
#endif

#include <string>

namespace ROSLITE_NAMESPACE
{

void init(int& argc, char** argv, const std::string& name);

void spin();

void spinOnce();

bool ok();

void generated_init();

}  // namespace ROSLITE_NAMESPACE
