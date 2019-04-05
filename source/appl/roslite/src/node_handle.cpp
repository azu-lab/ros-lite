#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/node_handle.h"
#else
  #include "ros/node_handle.h"
#endif

#include <limits>
#include <cstdio>
#include <string>

#define MIN(a, b) ((a) < (b) ? (a) : (b))

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

Subscriber NodeHandle::subscribe_(const std::string& t, uint32_t queue_size, std::shared_ptr<SubscriberInfo> subscriber_info)
{
  // roslite_debug_printf("[subscribe] topic: %s\n", topic.c_str());
  // roslite_debug_printf("[subscribe] count = [%d]\n", TIMap.count(topic));
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
  if (TIMap.count(topic) == 0){
    TIMapMutex.unlock();
    roslite_debug_printf("[ERROR] cannot find this topic [%s] ...\n", topic.c_str());
    ROSLITE_BREAK_ERROR();
    TIMapMutex.lock();
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

template <class T> T xml_cast(XmlRpc::XmlRpcValue xml_value) 
{
  return static_cast<T>(xml_value);
}

template <class T> bool xml_castable(int XmlType) 
{
  return false;
}

template<> bool xml_castable<std::string>(int XmlType)
{
  return XmlType == XmlRpc::XmlRpcValue::TypeString;
}

template<> bool xml_castable<double>(int XmlType)
{
  return ( 
      XmlType == XmlRpc::XmlRpcValue::TypeDouble ||
      XmlType == XmlRpc::XmlRpcValue::TypeInt ||
      XmlType == XmlRpc::XmlRpcValue::TypeBoolean );
}

template<> bool xml_castable<float>(int XmlType)
{
  return ( 
      XmlType == XmlRpc::XmlRpcValue::TypeDouble ||
      XmlType == XmlRpc::XmlRpcValue::TypeInt ||
      XmlType == XmlRpc::XmlRpcValue::TypeBoolean );
}

template<> bool xml_castable<int>(int XmlType)
{
  return ( 
      XmlType == XmlRpc::XmlRpcValue::TypeDouble ||
      XmlType == XmlRpc::XmlRpcValue::TypeInt ||
      XmlType == XmlRpc::XmlRpcValue::TypeBoolean );
}

template<> bool xml_castable<bool>(int XmlType)
{
  return ( 
      XmlType == XmlRpc::XmlRpcValue::TypeDouble ||
      XmlType == XmlRpc::XmlRpcValue::TypeInt ||
      XmlType == XmlRpc::XmlRpcValue::TypeBoolean );
}

template<> double xml_cast(XmlRpc::XmlRpcValue xml_value)
{
    using namespace XmlRpc;
    switch(xml_value.getType()) {
    case XmlRpc::XmlRpcValue::TypeDouble:
        return static_cast<double>(xml_value);
    case XmlRpc::XmlRpcValue::TypeInt:
        return static_cast<double>(static_cast<int>(xml_value));
    case XmlRpc::XmlRpcValue::TypeBoolean:
        return static_cast<double>(static_cast<bool>(xml_value));
    default:
        return 0.0;
    };
}

template<> float xml_cast(XmlRpc::XmlRpcValue xml_value)
{
    using namespace XmlRpc;
    switch(xml_value.getType()) {
    case XmlRpc::XmlRpcValue::TypeDouble:
        return static_cast<float>(static_cast<double>(xml_value));
    case XmlRpc::XmlRpcValue::TypeInt:
        return static_cast<float>(static_cast<int>(xml_value));
    case XmlRpc::XmlRpcValue::TypeBoolean:
        return static_cast<float>(static_cast<bool>(xml_value));
    default:
        return 0.0f;
    };
}

template<> int xml_cast(XmlRpc::XmlRpcValue xml_value)
{
    using namespace XmlRpc;
    switch(xml_value.getType()) {
    case XmlRpc::XmlRpcValue::TypeDouble:
        return static_cast<int>(static_cast<double>(xml_value));
    case XmlRpc::XmlRpcValue::TypeInt:
        return static_cast<int>(xml_value);
    case XmlRpc::XmlRpcValue::TypeBoolean:
        return static_cast<int>(static_cast<bool>(xml_value));
    default:
        return 0;
    };
}

template<> bool xml_cast(XmlRpc::XmlRpcValue xml_value)
{
    using namespace XmlRpc;
    switch(xml_value.getType()) {
    case XmlRpc::XmlRpcValue::TypeDouble:
        return static_cast<bool>(static_cast<double>(xml_value));
    case XmlRpc::XmlRpcValue::TypeInt:
        return static_cast<bool>(static_cast<int>(xml_value));
    case XmlRpc::XmlRpcValue::TypeBoolean:
        return static_cast<bool>(xml_value);
    default:
        return false;
    };
}

bool getParamImpl(const std::string& key, XmlRpc::XmlRpcValue& v)
{
    roslite_er_t roslite_ret;
    
    // Get ROS-param bridge tid
    roslite_id_t param_bridge_tid;
    for (;;)
    {
        param_bridge_tid = (roslite_id_t)roslite_name_lookup("ROSPARAM_TID", ROSLITE_NSID_GLOBAL);
        if (param_bridge_tid > 0)
        {
            break;
        }
        roslite_thread_delay(1);
    }

    // Send request to ROS param handler
    // message format :
    //   +--------------------+-------------+------------------------------------------------------+
    //   | key_length (4byte) | tid (2byte) | key string                                           |
    //   +--------------------+-------------+------------------------------------------------------+

    roslite_message_fd_t send_fd = roslite_message_connect(param_bridge_tid);
    if (send_fd < 0)
    {
        roslite_debug_printf("failed to connect\n");
        roslite_debug_break();
    }

    char buffer[64];
    uint32_t key_length = key.length();
    uint32_t sent_length = 0;
    for (int i = 0; ; ++i)
    {
        if (i == 0)
        {
            memcpy(buffer, &key_length, sizeof(key_length));
            
            roslite_erid_t tid = roslite_thread_getid();
            if (tid < 0)
            {
                roslite_debug_printf("roslite_thread_getid() error. ret=%d file=%s line=%d\n", tid, __FILE__, __LINE__);
                roslite_debug_break();
            }
            memcpy(buffer + sizeof(key_length), &tid, sizeof(tid));
            
            uint32_t current_length = MIN(key_length, sizeof(buffer) - sizeof(key_length) - sizeof(tid));
            memcpy(buffer + sizeof(key_length) + sizeof(tid), key.c_str(), current_length);
            sent_length = current_length;
        }
        else
        {
            uint32_t size = MIN(key_length - sent_length, sizeof(buffer));
            memcpy(buffer, key.c_str() + sent_length, size);
            sent_length += size;
        }

        roslite_ret = roslite_message_send(send_fd, buffer, sizeof(buffer), ROSLITE_MSG_SEND_ASYNC);
        if (roslite_ret != ROSLITE_EOK)
        {
            roslite_debug_printf("roslite_message_send() error. ret=%d file=%s line=%d\n", roslite_ret, __FILE__, __LINE__);
            roslite_debug_break();
        }

        if (sent_length == key_length)
        {
            break;
        }
    }

    roslite_message_close(send_fd);

    // Receive reply from ROS param handler
    // message format :
    //   +---------------------+--------------------------------------------------------------------+
    //   | data_length (4byte) | data string                                                        |
    //   +---------------------+--------------------------------------------------------------------+

    roslite_message_fd_t receive_fd = roslite_message_accept();
    if (receive_fd == -1)
    {
        roslite_debug_printf("failed to accept a connection\n");
        roslite_debug_break();
    }
    
    uint32_t data_length;
    char *data;
    uint32_t received_length;
    for (int i = 0; ; ++i)
    {
        uint32_t received_size = sizeof(buffer);
        roslite_ret = roslite_message_receive(receive_fd, buffer, &received_size, ROSLITE_MSG_RECV_SYNC);
        if (roslite_ret != ROSLITE_EOK)
        {
            roslite_debug_printf("roslite_message_receive() error. ret=%d file=%s line=%d\n", roslite_ret, __FILE__, __LINE__);
            roslite_debug_break();
        }

        if (received_size != sizeof(buffer))
        {
            roslite_debug_printf("unexpected size received. size=%d file=%s line=%d\n", received_size, __FILE__, __LINE__);
            roslite_debug_break();
        }

        if (i == 0)
        {
            memcpy(&data_length, buffer, sizeof(uint32_t));
            data = new char[data_length];
            uint32_t current_length = MIN(data_length, sizeof(buffer) - sizeof(uint32_t));
            memcpy(data, buffer + sizeof(uint32_t), current_length);
            received_length = current_length;
        }
        else
        {
            uint32_t current_length = MIN(data_length - received_length, sizeof(buffer));
            memcpy(data + received_length, buffer, current_length);
            received_length += current_length;
        }

        if (data_length == received_length)
        {
            break;
        }
    }

    roslite_message_close(receive_fd);

    if (data_length <= 0)
    {
        return false;
    }

    int offset = 0;
    bool ret = v.fromXml(data, &offset);
    delete[] data;
    if (!ret)
    {
        return false;
    }
    
    return true;
}
    
bool getParamImpl(const std::string& key, std::string& s)
{
    XmlRpc::XmlRpcValue v;
    if (!getParamImpl(key, v))
    {
        return false;
    }

    if (v.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
        return false;
    }

    s = std::string(v);
    return true;
}
    
bool getParamImpl(const std::string& key, double& d)
{
    XmlRpc::XmlRpcValue v;
    if (!getParamImpl(key, v))
    {
        return false;
    }

    if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
        d = (int)v;
    }
    else if (v.getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
        return false;
    }
    else
    {
        d = v;
    }

    return true;
}
    
bool getParamImpl(const std::string& key, int& i)
{
    XmlRpc::XmlRpcValue v;
    if (!getParamImpl(key, v))
    {
        return false;
    }

    if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble)
    {
        double d = v;

        if (fmod(d, 1.0) < 0.5)
        {
            d = floor(d);
        }
        else
        {
            d = ceil(d);
        }

        i = d;
    }
    else if (v.getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
        return false;
    }
    else
    {
        i = v;
    }

    return true;
}
    
bool getParamImpl(const std::string& key, bool& b)
{
    XmlRpc::XmlRpcValue v;
    if (!getParamImpl(key, v))
    {
        return false;
    }
    
    if (v.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
    {
        return false;
    }
    
    b = v;
    return true;
}

template <class T>
bool getParamImpl(const std::string& key, std::vector<T>& vec)
{
    XmlRpc::XmlRpcValue v;
    if (!getParamImpl(key, v))
    {
        return false;
    }

    if (v.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        return false;
    }

    vec.resize(v.size());

    for (int i = 0; i < v.size(); i++)
    {
        if (!xml_castable<T>(v[i].getType()))
        {
            return false;
        }

        vec[i] = xml_cast<T>(v[i]);
    }

    return true;
}

template <class T>
bool getParamImpl(const std::string& key, std::map<std::string, T>& map)
{
    XmlRpc::XmlRpcValue xml_value;
    if (!getParamImpl(key, xml_value))
    {
        return false;
    }

    if (xml_value.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
        return false;
    }

    for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = xml_value.begin();
         it != xml_value.end();
         ++it)
    {
        if (!xml_castable<T>(it->second.getType()))
        {
            return false;
        }
        map[it->first] = xml_cast<T>(it->second);
    }

    return true;
}

bool NodeHandle::getParam(const std::string& key, XmlRpc::XmlRpcValue& v) const
{
    return getParamImpl(key, v);
}
    
bool NodeHandle::getParam(const std::string& key, std::string& s) const
{
    return getParamImpl(key, s);
}
    
bool NodeHandle::getParam(const std::string& key, double& d) const
{
    return getParamImpl(key, d);
}
    
bool NodeHandle::getParam(const std::string& key, int& i) const
{
    return getParamImpl(key, i);
}
    
bool NodeHandle::getParam(const std::string& key, bool& b) const
{
    return getParamImpl(key, b);
}
    
bool NodeHandle::getParam(const std::string& key, std::vector<std::string>& vec) const
{
    return getParamImpl(key, vec);
}
    
bool NodeHandle::getParam(const std::string& key, std::vector<double>& vec) const
{
    return getParamImpl(key, vec);
}
    
bool NodeHandle::getParam(const std::string& key, std::vector<float>& vec) const
{
    return getParamImpl(key, vec);
}
    
bool NodeHandle::getParam(const std::string& key, std::vector<int>& vec) const
{
    return getParamImpl(key, vec);
}
    
bool NodeHandle::getParam(const std::string& key, std::vector<bool>& vec) const
{
    return getParamImpl(key, vec);
}
    
bool NodeHandle::getParam(const std::string& key, std::map<std::string, std::string>& map) const
{
    return getParamImpl(key, map);
}
    
bool NodeHandle::getParam(const std::string& key, std::map<std::string, double>& map) const
{
    return getParamImpl(key, map);
}
    
bool NodeHandle::getParam(const std::string& key, std::map<std::string, float>& map) const
{
    return getParamImpl(key, map);
}
    
bool NodeHandle::getParam(const std::string& key, std::map<std::string, int>& map) const
{
    return getParamImpl(key, map);
}
    
bool NodeHandle::getParam(const std::string& key, std::map<std::string, bool>& map) const
{
    return getParamImpl(key, map);
}

}  // namespace ROSLITE_NAMESPACE
