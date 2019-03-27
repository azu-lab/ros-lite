#pragma once

#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/common.h"
#else
  #include "ros/common.h"
#endif

namespace ROSLITE_NAMESPACE
{
    void entryNodeThread(uint32_t start_code, uintptr_t exinf);

    void createAndStartNodeThread(int (*node_main)(int, char**));
    
}  // namespace ROSLITE_NAMESPACE
