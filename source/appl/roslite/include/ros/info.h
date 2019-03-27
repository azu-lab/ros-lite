#pragma once

#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/common.h"
  #include "roslite/include/ros/serialization.h"
#else
  #include "ros/common.h"
  #include "ros/serialization.h"
#endif

#include <string>
#include <vector>
#include <memory>
#include <queue>

namespace ROSLITE_NAMESPACE
{

class SubscriberInfo
{
public:        
    SubscriberInfo(const std::string& recv_tname, const uint8_t assigned_hwclid, roslite_id_t main_tid);

    virtual void entryCallBack(SerializedMessage& s_message)=0;
    roslite_id_t getTid();
    std::string recv_tname_;
    uint8_t assigned_hwclid_;
    roslite_id_t main_tid_;
    std::queue<struct CallbackInfo> CallbackQueue;
    roslite_id_t getMainTid();
};

template<typename MessageType>
class SubscriberInfoImpl :  public SubscriberInfo
{
public:
    SubscriberInfoImpl(const std::string& recv_tname, void (*callBackFunc)(const std::shared_ptr<MessageType const>&), const uint8_t assigned_hwclid, roslite_id_t main_tid)
    : msg_( std::shared_ptr<MessageType>(new MessageType) )
    , callBackFunc_(callBackFunc)
    , SubscriberInfo(recv_tname, assigned_hwclid, main_tid)
    {}

    std::shared_ptr<MessageType> msg_;
    void (*callBackFunc_)(const std::shared_ptr<MessageType const>&);

    void entryCallBack(SerializedMessage& s_message){
        ROSLITE_NAMESPACE::serialization::IStream stream((uint8_t*)s_message.message_start, (uint32_t)*s_message.buf);
        ROSLITE_NAMESPACE::serialization::deserialize(stream, *(msg_));
        callBackFunc_(msg_);
    }
};

template<typename MessageType, typename T>
class SubscriberInfoWithInstanceImpl :  public SubscriberInfo
{
public:
    SubscriberInfoWithInstanceImpl(const std::string& recv_tname, void (T::*callBackFunc)(const std::shared_ptr<MessageType const>&), T* obj, const uint8_t assigned_hwclid, roslite_id_t main_tid)
    : msg_( std::shared_ptr<MessageType>(new MessageType) )
    , callBackFunc_(callBackFunc)
    , obj_(obj)
    , SubscriberInfo(recv_tname, assigned_hwclid, main_tid)
    {}

    std::shared_ptr<MessageType> msg_;
    void (T::*callBackFunc_)(const std::shared_ptr<MessageType const>&);
    T* obj_;

    void entryCallBack(SerializedMessage& s_message){
        ROSLITE_NAMESPACE::serialization::IStream stream((uint8_t*)s_message.message_start, (uint32_t)*s_message.buf);
        ROSLITE_NAMESPACE::serialization::deserialize(stream, *(msg_));
        (obj_->*callBackFunc_)(msg_);
    }
};

class TopicInfo
{
public:
    TopicInfo();

    std::vector<std::shared_ptr<SubscriberInfo>> SIs;
    uint32_t get_cluster_index();
private:
    uint32_t cluster_index_ = 0;

};
// typedef std::vector<std::shared_ptr<SubscriberInfo>> TopicInfo;

struct CallbackInfo
{
public:
    std::string topic;
    uint32_t start_code;
    SerializedMessage s_message;
    uint64_t reg_time;
};

struct MessageReceiveArgs
{
public:
    std::string topic;
    uint32_t queue_size;
};

struct PublisherSendObject
{
public:
    int size;
    uint8_t * address;
    int segment_id;
};

}  // namespace ROSLITE_NAMESPACE
