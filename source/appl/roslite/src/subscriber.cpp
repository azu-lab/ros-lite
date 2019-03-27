#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/generated/globals.h"
  #include "roslite/include/ros/init.h"
  #include "roslite/include/ros/subscriber.h"
  #include "roslite/include/ros/debug.h"
#else
  #include "ros/generated/globals.h"
  #include "ros/init.h"
  #include "ros/subscriber.h"
  #include "ros/debug.h"
#endif

#include <string>

namespace ROSLITE_NAMESPACE
{

void messageReceive(uint32_t start_code, uintptr_t exinf)
{    
    MessageReceiveArgs* message_receive_args = (MessageReceiveArgs*) exinf;
    std::string topic = message_receive_args->topic;
    uint32_t queue_size = message_receive_args->queue_size;

    delete message_receive_args;

    TIMapMutex.lock();
    printThreadinfo((TIMap.at(topic).SIs[start_code])->recv_tname_);
    TIMapMutex.unlock();

    SerializedMessage s_message;

    while (ROSLITE_NAMESPACE::ok()) {
        /* Receive data size */
        uint32_t recv_size = 4;

        roslite_message_fd_t fd = roslite_message_accept();
        if (fd == -1) {
            // roslite_debug_printf("failed to accept a connection\n");
            continue;
        }

        SerializedMessage s_message;

        roslite_er_t ret;

        /* Receive message contents */
        ret = roslite_message_receive(fd, &(s_message.num_bytes), &recv_size, ROSLITE_MSG_RECV_SYNC);
        ROSLITE_CHECK_MESSAGE_RECEIVE(ret, recv_size, 4);
        // roslite_debug_printf("num_bytes: %d\n", s_message.num_bytes);

        s_message.buf = (uint8_t*)malloc(s_message.num_bytes);  
        s_message.message_start = (uint8_t*)(s_message.buf + 4);
        recv_size = s_message.num_bytes;

        ret = roslite_message_receive(fd, s_message.buf, &recv_size, ROSLITE_MSG_RECV_SYNC);
        ROSLITE_CHECK_MESSAGE_RECEIVE(ret, recv_size, s_message.num_bytes);

        roslite_message_close(fd);

        // printSerializedMessage(s_message);

        TIMapMutex.lock();
        (TIMap.at(topic).SIs[start_code])->entryCallBack(s_message);
        TIMapMutex.unlock();
        free(s_message.buf);
    }
}

}  // namespace ROSLITE_NAMESPACE

