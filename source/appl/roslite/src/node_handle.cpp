#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/node_handle.h"
#else
  #include "ros/node_handle.h"
#endif

#include <limits>
#include <cstdio>
#include <string>

namespace ROSLITE_NAMESPACE
{

std::string to_string(uint32_t val)
{
  char buffer[std::numeric_limits<uint32_t>::digits10 + 2]; // '-' + NULL
  std::sprintf(buffer, "%d", val);
  return buffer;
}
  
std::string replace(std::string& stream, const std::string& target, const std::string& replacement)
{
  if (!target.empty()) {
    std::string::size_type pos = 0;
    while ((pos = stream.find(target, pos)) != std::string::npos) {
      stream.replace(pos, target.length(), replacement);
      pos += replacement.length();
    }
  }
  return stream;
}

Subscriber NodeHandle::subscribe_(const std::string& topic, uint32_t queue_size, std::shared_ptr<SubscriberInfo> subscriber_info)
{
  // roslite_debug_printf("[subscribe] topic: %s\n", topic.c_str());
  // roslite_debug_printf("[subscribe] count = [%d]\n", TIMap.count(topic));
  TIMapMutex.lock();
  if (TIMap.count(topic) == 0){
    roslite_debug_printf("[ERROR] cannot find this topic [%s] ...\n", topic.c_str());
    ROSLITE_BREAK_ERROR();
  }

  uint32_t cluster_index = TIMap.at(topic).get_cluster_index();
  TIMap.at(topic).SIs.push_back(subscriber_info);
  uint32_t SI_index = TIMap.at(topic).SIs.size() - 1;
  std::string recv_tname = topic, udb("_");
  recv_tname = replace(recv_tname, "/", "_") + udb + to_string((uint32_t)ROSLITE_TARGET_CLUSTER_ID) + udb + to_string(cluster_index);
  TIMap.at(topic).SIs[SI_index]->recv_tname_ = recv_tname;
  TIMapMutex.unlock();

  /* crete thread */
  roslite_thread_attr_t attr;
  roslite_id_t tid;
  roslite_er_t ret;

  MessageReceiveArgs* message_receive_args = new MessageReceiveArgs();
  message_receive_args->topic = topic;
  message_receive_args->queue_size = queue_size;

  roslite_thread_attr_init(&attr);
  // TODO
  // roslite_thread_attr_setlcid(&attr, ROSLITE_LCID_ANY);
  // roslite_thread_attr_setstacksize(&attr, 1024 * 10);
  tid = roslite_thread_create(&attr, messageReceive, (uintptr_t) message_receive_args);
  ROSLITE_CHECK_SIMPLE(tid);
  // roslite_debug_printf("[subscribe] recv_tname: %s, tid=%d\n", recv_tname.c_str(), tid);
  ret = roslite_name_register(recv_tname.c_str(), (roslite_generic_id_t)tid, ROSLITE_NSID_GLOBAL);
  ROSLITE_CHECK_SIMPLE(ret);

  ret = roslite_thread_start(tid, SI_index);
  ROSLITE_CHECK_SIMPLE(ret);

  ROSLITE_NAMESPACE::Subscriber subscriber;
  return subscriber;
}

}  // namespace ROSLITE_NAMESPACE
