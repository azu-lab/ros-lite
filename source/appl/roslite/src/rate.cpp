#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/rate.h"
#else
  #include "ros/rate.h"
#endif

#include <math.h>
#include <time.h>

namespace ROSLITE_NAMESPACE
{

Rate::Rate(double frequency)
    : expected_cycle_time_(1.0 / frequency)
    {}

bool Rate::sleep(){
    double integer_expected_cycle_time;
    double decimal_expected_cycle_time = modf(expected_cycle_time_, &integer_expected_cycle_time);
    struct timespec req;
    req.tv_sec = integer_expected_cycle_time;
    req.tv_nsec = decimal_expected_cycle_time * 1000000000; /* 500,000,000ナノ秒 = 500,000マイクロ秒 = 500ミリ秒 = 0.5秒 */
    
    nanosleep(&req, NULL);

    return true;
}

}  // namespace ROSLITE_NAMESPACE
