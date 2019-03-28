#include <ros/ros.h>
#include "roslite/roslite.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))

static void receive_request(roslite_id_t fd, std::string *key, roslite_id_t *tid)
{
    roslite_er_t ret;

    // Receive request from ROS-lite node
    // message format :
    //   +--------------------+-------------+------------------------------------------------------+
    //   | key_length (4byte) | tid (2byte) | key string                                           |
    //   +--------------------+-------------+------------------------------------------------------+
    char buffer[64];
    uint32_t key_length;
    std::vector<char> key_buffer;
    for (int i = 0; ; ++i)
    {
        uint32_t received_size = sizeof(buffer);
        ret = roslite_message_receive(fd, buffer, &received_size, ROSLITE_MSG_RECV_SYNC);
        if (ret != ROSLITE_EOK)
        {
            roslite_debug_printf("roslite_message_receive() error. ret=%d file=%s line=%d\n", ret, __FILE__, __LINE__);
            roslite_debug_break();
        }

        if (received_size != sizeof(buffer))
        {
            roslite_debug_printf("unexpected size received. size=%d file=%s line=%d\n", received_size, __FILE__, __LINE__);
            roslite_debug_break();
        }

        if (i == 0)
        {
            memcpy(&key_length, buffer, sizeof(uint32_t));
            memcpy(tid, buffer + sizeof(key_length), sizeof(roslite_id_t));
            key_buffer.reserve(key_length);

            uint32_t data_length = MIN(key_length, sizeof(buffer) - sizeof(uint32_t) - sizeof(roslite_id_t));
            char *key_head = buffer + sizeof(uint32_t) + sizeof(roslite_id_t);
            key_buffer.insert(key_buffer.end(), key_head, key_head + data_length);
        }
        else
        {
            uint32_t data_length = MIN(key_length - key_buffer.size(), sizeof(buffer));
            key_buffer.insert(key_buffer.end(), buffer, buffer + data_length);
        }

        if (key_buffer.size() == key_length)
        {
            break;
        }
    }

    *key = std::string(key_buffer.begin(), key_buffer.end());
}

static void send_reply(roslite_id_t fd, bool is_value_valid, const XmlRpc::XmlRpcValue &value)
{
    roslite_er_t ret;

    // Send reply to ROS-lite node
    // message format :
    //   +---------------------+--------------------------------------------------------------------+
    //   | data_length (4byte) | data string                                                        |
    //   +---------------------+--------------------------------------------------------------------+
    char buffer[64];
    if (is_value_valid)
    {
        std::string data = value.toXml();
        uint32_t data_length = data.length();
        uint32_t sent_length = 0;
        for (int i = 0; ; ++i)
        {
            if (i == 0)
            {
                memcpy(buffer, &data_length, sizeof(data_length));

                uint32_t current_length = MIN(data_length, sizeof(buffer) - sizeof(data_length));
                memcpy(buffer + sizeof(data_length), data.c_str(), current_length);
                sent_length = current_length;
            }
            else
            {
                uint32_t size = MIN(data_length - sent_length, sizeof(buffer));
                memcpy(buffer, data.c_str() + sent_length, size);
                sent_length += size;
            }

            ret = roslite_message_send(fd, buffer, sizeof(buffer), ROSLITE_MSG_SEND_ASYNC);
            if (ret != ROSLITE_EOK)
            {
                roslite_debug_printf("roslite_message_send() error. ret=%d file=%s line=%d\n", ret, __FILE__, __LINE__);
                roslite_debug_break();
            }

            if (sent_length == data_length)
            {
                break;
            }
        }
    }
    else
    {
        uint32_t data_length = 0;
        memcpy(buffer, &data_length, sizeof(uint32_t));
        ret = roslite_message_send(fd, buffer, sizeof(buffer), ROSLITE_MSG_SEND_ASYNC);
        if (ret != ROSLITE_EOK)
        {
            roslite_debug_printf("roslite_message_send() error. ret=%d file=%s line=%d\n", ret, __FILE__, __LINE__);
            roslite_debug_break();
        }
    }
}

static void handle_param_request(ros::NodeHandle &ros_nh)
{
    roslite_er_t ret;

    // register this thread as ROS-param bridge thread
    ret = roslite_name_register("ROSPARAM_TID", (roslite_generic_id_t)roslite_thread_getid(), ROSLITE_NSID_GLOBAL);
    if (ret != ROSLITE_EOK)
    {
        roslite_debug_printf("roslite_name_register() error. ret=%d file=%s line=%d\n", ret, __FILE__, __LINE__);
        roslite_debug_break();
    }

    for (;;)
    {
        // Receive request form ROS-lite node
        roslite_message_fd_t receive_fd = roslite_message_accept();
        if (receive_fd == -1)
        {
            roslite_debug_printf("failed to accept a connection\n");
            roslite_debug_break();
        }

        std::string key;
        roslite_id_t tid;
        receive_request(receive_fd, &key, &tid);

        roslite_message_close(receive_fd);

        // Get ROS-param
        XmlRpc::XmlRpcValue value;
        bool is_value_valid = false;
        if (ros_nh.hasParam(key))
        {
            if (ros_nh.getParam(key, value))
            {
                is_value_valid = true;
            }
        }

        // send reply to ROS-lite node
        roslite_message_fd_t send_fd = roslite_message_connect(tid);
        if (send_fd < 0)
        {
            roslite_debug_printf("failed to connect\n");
            roslite_debug_break();
        }

        send_reply(send_fd, is_value_valid, value);

        roslite_message_close(send_fd);
    }
}

void cluster0_main()
{
    char *argv = (char *)"ros_param_bridge";
    int argc = 1;

    ros::init(argc, &argv, "roslite_param_bridge", ros::init_options::NoSigintHandler);

    ros::NodeHandle ros_nh;

    handle_param_request(ros_nh);
}
