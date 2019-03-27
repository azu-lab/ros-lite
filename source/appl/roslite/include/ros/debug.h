#pragma once

#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/common.h"
  #include "roslite/include/ros/serialization.h"
#else
  #include "ros/common.h"
  #include "ros/serialization.h"
#endif

#include <string>

// 
// ERROR CHECK
// 

#define ROSLITE_CHECK_SIMPLE(arg) \
    if (arg < 0) \
    { \
        roslite_thread_info_t thread_ref; \
        roslite_thread_get_info(roslite_thread_getid(), &thread_ref); \
        roslite_debug_printf("[ERROR]: arg=%d, hwclid=%d, lcid=%d, tid=%d\n", arg, ROSLITE_TARGET_CLUSTER_ID, thread_ref.lcid, roslite_thread_getid() ); \
        roslite_debug_printf("         file=%s \n", __FILE__); \
        roslite_debug_printf("         line=%d \n", __LINE__); \
        roslite_debug_break(); \
    }

#define ROSLITE_CHECK_MESSAGE_SEND(ret) \
    if (ret < 0){ \
        roslite_debug_printf("[ERROR] message send error: %d \n", ret);  \
        roslite_debug_printf("         file=%s \n", __FILE__); \
        roslite_debug_printf("         line=%d \n", __LINE__); \
    }

#define ROSLITE_CHECK_MESSAGE_RECEIVE(ret, output_recv_size, input_recv_size) \
    if (ret < 0){ \
        roslite_debug_printf("[ERROR] message receive error: %d\n", ret); \
        roslite_debug_printf("         file=%s \n", __FILE__); \
        roslite_debug_printf("         line=%d \n", __LINE__); \
    } \
    if (output_recv_size != input_recv_size){ \
        roslite_debug_printf("[ERROR] message receive size error: %d\n", ret); \
        roslite_debug_printf("         file=%s \n", __FILE__); \
        roslite_debug_printf("         line=%d \n", __LINE__); \
    }

#define ROSLITE_BREAK_ERROR() \
    roslite_thread_info_t thread_ref; \
    roslite_thread_get_info(roslite_thread_getid(), &thread_ref); \
    roslite_debug_printf("[ERROR]: hwclid=%d, lcid=%d, tid=%d\n", ROSLITE_TARGET_CLUSTER_ID, thread_ref.lcid, roslite_thread_getid() ); \
    roslite_debug_break(); 

namespace ROSLITE_NAMESPACE
{

// 
// PRINT 
// 

void printThreadinfo(const std::string& exinf);

void printSerializedMessage(SerializedMessage s_message);  // for std_msgs::String

}  // namespace ROSLITE_NAMESPACE
