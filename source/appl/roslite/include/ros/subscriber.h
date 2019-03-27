#pragma once

#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/common.h"
#else
  #include "ros/common.h"
#endif

// #include "init.h"

namespace ROSLITE_NAMESPACE
{

// template<typename M>
class Subscriber
{
public:
    Subscriber(){};
    ~Subscriber(){};
};

void messageReceive(uint32_t start_code, uintptr_t exinf);

}  // namespace ROSLITE_NAMESPACE

