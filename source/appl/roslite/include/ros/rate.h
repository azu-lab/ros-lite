#pragma once

#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/common.h"
#else
  #include "ros/common.h"
#endif

#include <math.h>
#include <time.h>

namespace ROSLITE_NAMESPACE
{

class Rate
{
public:
    Rate(double frequency);

    bool sleep();

private:
    double expected_cycle_time_;
};

}  // namespace ROSLITE_NAMESPACE
