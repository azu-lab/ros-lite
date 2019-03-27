#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/time.h"
#else
  #include "ros/time.h"
#endif

#include <climits>
#include <stdexcept>

namespace ROSLITE_NAMESPACE
{

void normalizeSecNSec(uint64_t& sec, uint64_t& nsec)
{
uint64_t nsec_part = nsec % 1000000000UL;
uint64_t sec_part = nsec / 1000000000UL;

if (sec + sec_part > UINT_MAX)
    throw std::runtime_error("Time is out of dual 32-bit range");

sec += sec_part;
nsec = nsec_part;
}

void normalizeSecNSec(uint32_t& sec, uint32_t& nsec)
{
uint64_t sec64 = sec;
uint64_t nsec64 = nsec;

normalizeSecNSec(sec64, nsec64);

sec = (uint32_t)sec64;
nsec = (uint32_t)nsec64;
}

}  // namespace ROSLITE_NAMESPACE
