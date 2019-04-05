#pragma once

#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/generated/main_replacer.h"
  #include "roslite/include/ros/common.h"
  #include "roslite/include/ros/info.h"
  #include "roslite/include/ros/init.h"
  #include "roslite/include/ros/node_handle.h"
  #include "roslite/include/ros/publisher.h"
  #include "roslite/include/ros/subscriber.h"
  #include "roslite/include/ros/rate.h"
  #include "roslite/include/ros/serialization.h"
  #include "roslite/include/ros/time.h"
  #include "roslite/include/ros/duration.h"
  #include "roslite/include/ros/thread.h"
  #include "roslite/include/ros/debug.h"
#else
  #include "ros/generated/main_replacer.h"
  #include "ros/common.h"
  #include "ros/info.h"
  #include "ros/init.h"
  #include "ros/node_handle.h"
  #include "ros/publisher.h"
  #include "ros/subscriber.h"
  #include "ros/rate.h"
  #include "ros/serialization.h"
  #include "ros/time.h"
  #include "ros/duration.h"
  #include "ros/thread.h"
  #include "ros/debug.h"

  #define ROS_INFO(...) \
      roslite_debug_printf(__VA_ARGS__); \
      roslite_debug_printf("\n");
  #define ROS_WARN(...) \
      roslite_debug_printf(__VA_ARGS__); \
      roslite_debug_printf("\n");
  #define ROS_ERROR(...) \
      roslite_debug_printf(__VA_ARGS__); \
      roslite_debug_printf("\n");
  #define ROS_INFO_STREAM(args)
  #define ROS_ERROR_STREAM(args)
#endif

#include <stdint.h>
#include <cstddef>
#include <cfloat>
#include <cmath>
#include <algorithm>
