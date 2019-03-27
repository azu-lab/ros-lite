#pragma once

#include "roslite/roslite.h"

#if ROSLITE_TARGET_CLUSTER_ID == 0
  #define ROSLITE_NAMESPACE roslite
#else
  #define ROSLITE_NAMESPACE ros
#endif
