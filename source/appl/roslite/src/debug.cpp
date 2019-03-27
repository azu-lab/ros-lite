#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/debug.h"
#else
  #include "ros/debug.h"
#endif

#include <string>

// 
// PRINT 
// 
namespace ROSLITE_NAMESPACE
{

void printThreadinfo(const std::string& exinf)
{
    roslite_thread_info_t thread_ref;
    roslite_thread_get_info(roslite_thread_getid(), &thread_ref);
    roslite_debug_printf("[%s] hwclid=%d, lcid=%d, tid=%d\n", exinf.c_str(), ROSLITE_TARGET_CLUSTER_ID, thread_ref.lcid, roslite_thread_getid());
}

void printSerializedMessage(SerializedMessage s_message)  // for std_msgs::String
{
    roslite_debug_printf("SerializedMessage s_message:\n  buf:0x%x\n  buf:%d\n  num_bytes:%d\n  message_start:0x%x\n  message_start:%s\n", s_message.buf, (uint32_t)*s_message.buf, (int) s_message.num_bytes, s_message.message_start, (char*) s_message.message_start + 4);
}

}  // namespace ROSLITE_NAMESPACE
