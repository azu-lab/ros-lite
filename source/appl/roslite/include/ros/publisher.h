#pragma once

#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/generated/globals.h"
  #include "roslite/include/ros/common.h"
  #include "roslite/include/ros/debug.h"
#else
  #include "ros/generated/globals.h"
  #include "ros/common.h"
  #include "ros/debug.h"
#endif

#include <string>

namespace ROSLITE_NAMESPACE
{

class Publisher
{
public:
    Publisher(){};
    ~Publisher(){};
    std::string topic_name_;

    void publish_internal(ROSLITE_NAMESPACE::SerializedMessage s_message) const;

    template <typename M>
    void publish(const M& message) const
    {
        ROSLITE_NAMESPACE::SerializedMessage s_message = ROSLITE_NAMESPACE::serialization::serializeMessage(message);

        publish_internal(s_message);

        free(s_message.buf);
    }

};  // class Publisher

}  // namespace ROSLITE_NAMESPACE

