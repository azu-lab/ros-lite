#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/init.h"
  #include "roslite/include/ros/init.h"
  #include "roslite/include/ros/debug.h"
  #include "roslite/include/ros/generated/globals.h"
#else
  #include "ros/init.h"
  #include "ros/init.h"
  #include "ros/debug.h"
  #include "ros/generated/globals.h"
#endif

#include <string>
#include <queue>
#include <algorithm>

namespace ROSLITE_NAMESPACE
{

void init(int& argc, char** argv, const std::string& name)
{
    roslite_id_t tid = roslite_thread_getid();
    roslite_er_t ret;

    std::string tname("_main");
    tname = name + tname;
    // roslite_debug_printf("[init ] tname: %s\n", tname.c_str());

    printThreadinfo(tname);
    
    ret = roslite_name_register(tname.c_str(), (roslite_generic_id_t)tid, ROSLITE_NSID_GLOBAL);
    if (ret != ROSLITE_EOK)
    {
        roslite_debug_printf("error: roslite_name_register: ret=%d\n", ret);
        roslite_debug_break();
    }
}

void spin()
{
    while(ROSLITE_NAMESPACE::ok())
    {
        spinOnce();
        roslite_thread_delay(0);
    }
}

static bool compareCallbackInfo(const CallbackInfo& left, const CallbackInfo& right)
{
    return (left.reg_time < right.reg_time);
}

void spinOnce()
{
    /* get self thread ID */
    roslite_id_t tid = roslite_thread_getid();
    //std::vector<std::queue<struct CallbackInfo>*> queue_list;
    std::vector<struct CallbackInfo> processing_list;

    /* get all queue list */
    TIMapMutex.lock();
    for(std::map<std::string, TopicInfo>::iterator itrTIMap = TIMap.begin(); itrTIMap != TIMap.end(); ++itrTIMap)
    {
        TopicInfo &ti = itrTIMap->second;
        for(std::vector<std::shared_ptr<SubscriberInfo> >::iterator itr = ti.SIs.begin(); itr != ti.SIs.end(); ++itr)
        {
            roslite_id_t subscriber_tid = (*itr)->getMainTid();
            if (tid == subscriber_tid)
            {
                while(!(*itr)->CallbackQueue.empty())
                {
                    processing_list.push_back((*itr)->CallbackQueue.front());
                    (*itr)->CallbackQueue.pop();
                }
            }
        }
    }
    TIMapMutex.unlock();

    std::sort(processing_list.begin(), processing_list.end(), compareCallbackInfo);

    for(std::vector<struct CallbackInfo>::iterator itr = processing_list.begin(); itr != processing_list.end(); ++itr)
    {
        std::string topic = itr->topic;
        uint32_t start_code = itr->start_code;
        SerializedMessage s_message = itr->s_message;
        /* execute callback */
        TIMapMutex.lock();
        std::shared_ptr<SubscriberInfo> si = (TIMap.at(topic).SIs[start_code]);
        TIMapMutex.unlock();
        si->entryCallBack(s_message);
        /* free s_message */
        free(s_message.buf);
    }
}

bool ok()
{
  return true;
}

}  // namespace ROSLITE_NAMESPACE

