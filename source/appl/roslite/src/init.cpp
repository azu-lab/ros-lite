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
        roslite_thread_delay(10);
    }
}

void spinOnce()
{
    /* get self thread ID */
    roslite_id_t tid = roslite_thread_getid();
    std::vector <std::queue<struct CallbackInfo>*> queue_list;

    /* get all queue list */
    TIMapMutex.lock();
    for(auto topic_info : TIMap)
    {
        TopicInfo &ti = topic_info.second;
        for(auto itr = ti.SIs.begin(); itr != ti.SIs.end(); ++itr)
        {
            roslite_id_t subscriber_tid = (*itr)->getMainTid();
            if (tid == subscriber_tid)
            {
                if (!((*itr)->CallbackQueue.empty()))
                {
                    queue_list.push_back(&(*itr)->CallbackQueue);
                }
            }
        }
    }
    TIMapMutex.unlock();

    /* queue is empty */
    if (queue_list.empty())
    {
        return;
    }
    /* search oldest entry */
    std::queue<struct CallbackInfo>* oldest_queue = NULL;
    uint64_t oldest_time = UINT64_MAX;
    for(auto itr = queue_list.begin(); itr != queue_list.end(); ++itr)
    {
        CallbackInfo &tmp_info = (*itr)->front();
        time_t tmp_time = tmp_info.reg_time;

        if (tmp_time <= oldest_time)
        {
            oldest_time = tmp_time;
            oldest_queue = *itr;
        }
    }
    std::string topic = oldest_queue->front().topic;
    uint32_t start_code = oldest_queue->front().start_code;
    SerializedMessage s_message = oldest_queue->front().s_message;
    /* execute callback */
    TIMapMutex.lock();
    std::shared_ptr<SubscriberInfo> si = (TIMap.at(topic).SIs[start_code]);
    TIMapMutex.unlock();
    si->entryCallBack(s_message);
    /* free s_message */
    free(s_message.buf);


    oldest_queue->pop();
}

bool ok()
{
  return true;
}

}  // namespace ROSLITE_NAMESPACE

