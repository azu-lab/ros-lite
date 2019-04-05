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

#include "XmlRpcValue.h"

#include <string>
#include <memory>

namespace ROSLITE_NAMESPACE
{

std::string to_string(uint32_t val);

std::string replace(std::string& stream, const std::string& target, const std::string& replacement);

class NodeHandle
{
public:

NodeHandle(const std::string &ns = std::string()) {}
    
template <class MessageType>
Publisher advertise(const std::string& t, uint32_t queue_size, bool latch=false)
{
    // roslite_debug_printf("[advertise] count = [%d]\n", TIMap.count(topic));
    std::string topic;
    if ((t.length() > 0) && (t[0] == '/'))
    {
        topic = t;
    }
    else
    {
        topic = '/';
        topic += t;
    }

    TIMapMutex.lock();
    int count = TIMap.count(topic);
    TIMapMutex.unlock();
    if (count == 0){
        roslite_debug_printf("[ERROR] cannot find this topic [%s] ...\n", topic.c_str());
        ROSLITE_BREAK_ERROR();
    }
    
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

bool getParam(const std::string& key, XmlRpc::XmlRpcValue& v) const;
bool getParam(const std::string& key, std::string& s) const;
bool getParam(const std::string& key, double& d) const;
bool getParam(const std::string& key, int& i) const;
bool getParam(const std::string& key, bool& b) const;
bool getParam(const std::string& key, std::vector<std::string>& vec) const;
bool getParam(const std::string& key, std::vector<double>& vec) const;
bool getParam(const std::string& key, std::vector<float>& vec) const;
bool getParam(const std::string& key, std::vector<int>& vec) const;
bool getParam(const std::string& key, std::vector<bool>& vec) const;
bool getParam(const std::string& key, std::map<std::string, std::string>& map) const;
bool getParam(const std::string& key, std::map<std::string, double>& map) const;
bool getParam(const std::string& key, std::map<std::string, float>& map) const;
bool getParam(const std::string& key, std::map<std::string, int>& map) const;
bool getParam(const std::string& key, std::map<std::string, bool>& map) const;

template<typename T>
void param(const std::string& param_name, T& param_val, const T& default_val) const
{
    if (getParam(param_name, param_val))
    {
        return;
    }
    
    param_val = default_val;
}


private:

Subscriber subscribe_(const std::string& topic, uint32_t queue_size, std::shared_ptr<SubscriberInfo> subscriber_info);

};  // class NodeHandle

}  // namespace ROSLITE_NAMESPACE
