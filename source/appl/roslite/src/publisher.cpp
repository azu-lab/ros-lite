#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/publisher.h"
#else
  #include "ros/publisher.h"
#endif

namespace ROSLITE_NAMESPACE
{

void Publisher::publish_internal(ROSLITE_NAMESPACE::SerializedMessage s_message) const
{
    roslite_id_t tid = 0;
    roslite_er_t ret;

    // roslite_debug_printf("[publish] topic_name_: %s, #subscriber = %d\n", topic_name_.c_str(), TIMap.at(topic_name_).size());
    TIMapMutex.lock();
    std::vector<std::shared_ptr<SubscriberInfo>> SIs_copy = TIMap.at(topic_name_).SIs;
    TIMapMutex.unlock();
    for(auto& SI_ptr : SIs_copy) {
        // roslite_debug_printf("[publish] recv_tname_: %s\n",  (*SI_ptr).recv_tname_.c_str());
        // roslite_debug_printf("[publish] assigned_hwclid_: %d\n",  (*SI_ptr).assigned_hwclid_);

        TIMapMutex.lock();
        tid = (*SI_ptr).getTid();
        std::string recv_tname = (*SI_ptr).recv_tname_;
        TIMapMutex.unlock();
        // roslite_debug_printf("[publish] tid: %d\n", tid);
        if(tid < 0) {
            roslite_debug_printf("[WARNING] cannot find subscriber [%s] ...\n", recv_tname.c_str());
        } else {
            roslite_message_fd_t fd = roslite_message_connect(tid);
            if(fd == -1) {
                roslite_debug_printf("[WARNING] cannot connect subscriber [%s] ...\n", recv_tname.c_str());
            } else {
                ret = roslite_message_send(fd, &(s_message.num_bytes), (int) 4, ROSLITE_MSG_SEND_ASYNC);
                ROSLITE_CHECK_MESSAGE_SEND(ret);

                ret = roslite_message_send(fd, s_message.buf, (int) s_message.num_bytes, ROSLITE_MSG_SEND_ASYNC);
                ROSLITE_CHECK_MESSAGE_SEND(ret);

                roslite_message_close(fd);
            }
        }
    }

}


}
