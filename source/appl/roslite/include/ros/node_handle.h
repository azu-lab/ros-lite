#pragma once

#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/common.h"
  #include "roslite/include/ros/publisher.h"
  #include "roslite/include/ros/subscriber.h"
#else
  #include "ros/common.h"
  #include "ros/publisher.h"
  #include "ros/subscriber.h"
#endif

#include <string>
#include <memory>

namespace ROSLITE_NAMESPACE
{

std::string to_string(uint32_t val);

std::string replace(std::string& stream, const std::string& target, const std::string& replacement);

class NodeHandle
{
public:

template <class MessageType>
Publisher advertise(const std::string& topic, uint32_t queue_size)
{
    // roslite_debug_printf("[advertise] count = [%d]\n", TIMap.count(topic));
    TIMapMutex.lock();
    if (TIMap.count(topic) == 0){
        roslite_debug_printf("[ERROR] cannot find this topic [%s] ...\n", topic.c_str());
        ROSLITE_BREAK_ERROR();
    }
    TIMapMutex.unlock();
    
    ROSLITE_NAMESPACE::Publisher publisher;
    publisher.topic_name_ = topic;
    return publisher;
}

template<class M>
Subscriber subscribe(const std::string& topic, uint32_t queue_size, void (*func)(const std::shared_ptr<M const>&))
{
    return subscribe_(topic, queue_size, std::shared_ptr<SubscriberInfo>(new SubscriberInfoImpl<M>("", func, ROSLITE_TARGET_CLUSTER_ID, roslite_thread_getid())));
}

template<class M, class T>
Subscriber subscribe(const std::string& topic, uint32_t queue_size, void (T::*fp)(const std::shared_ptr<M const>&), T *obj)
{
    return subscribe_(topic, queue_size, std::shared_ptr<SubscriberInfo>(new SubscriberInfoWithInstanceImpl<M, T>("", fp, obj, ROSLITE_TARGET_CLUSTER_ID, roslite_thread_getid())));
}


private:

Subscriber subscribe_(const std::string& topic, uint32_t queue_size, std::shared_ptr<SubscriberInfo> subscriber_info);

};  // class NodeHandle

}  // namespace ROSLITE_NAMESPACE
