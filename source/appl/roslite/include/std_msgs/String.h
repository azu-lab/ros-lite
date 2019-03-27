// [note] Auto-generated file
// [note] 2019-03-20T08:43:41Z
// [note] based on source/appl/ros_src/msg/std_msgs/String.msg
#ifndef ROSLITE_STD_MSGS_MESSAGE_STRING_H
#define ROSLITE_STD_MSGS_MESSAGE_STRING_H

#include <string>
#include <vector>
#include <map>
#include <ostream>

#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/common.h"
  #include "roslite/include/ros/serialization.h"
  #include "std_msgs/String.h"
#else
  #include "ros/common.h"
  #include "ros/serialization.h"
#endif

#if ROSLITE_TARGET_CLUSTER_ID == 0
  #define NAMESPACE_std_msgs roslite_std_msgs
#else
  #define NAMESPACE_std_msgs std_msgs
#endif

namespace NAMESPACE_std_msgs
{
template <class ContainerAllocator>
struct String_ {
  typedef String_<ContainerAllocator> Type;

  String_()
  : data()
  {
  }

  String_(const ContainerAllocator& _alloc)
  : data(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _data_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  data;


  typedef ::std::shared_ptr< ::NAMESPACE_std_msgs::String_<ContainerAllocator> > Ptr;
  typedef ::std::shared_ptr< ::NAMESPACE_std_msgs::String_<ContainerAllocator>  const> ConstPtr;

#if ROSLITE_TARGET_CLUSTER_ID == 0
  static Type FromRosMsg(const typename std_msgs::String_<ContainerAllocator>& ros_msg) {
    Type roslite_msg;
    roslite_msg.data = ros_msg.data;
    return roslite_msg;
  }

  typename std_msgs::String_<ContainerAllocator> ToRosMsg() const {
    typename std_msgs::String_<ContainerAllocator> ros_msg;
    ros_msg.data = data;
    return ros_msg;
  }
  
  static typename Type::ConstPtr FromRosMsgPtr(const typename std_msgs::String_<ContainerAllocator>::ConstPtr& ros_msg) {
    typename Type::Ptr roslite_msg(new Type());
    *roslite_msg = FromRosMsg(*ros_msg);
    return roslite_msg;
  }

  typename std_msgs::String_<ContainerAllocator>::ConstPtr ToRosMsgPtr() const {
    typename std_msgs::String_<ContainerAllocator>::Ptr ros_msg(new std_msgs::String_<ContainerAllocator>());
    *ros_msg = ToRosMsg();
    return ros_msg;
  }
#endif

}; // struct String
typedef  ::NAMESPACE_std_msgs::String_<std::allocator<void> > String;

typedef ::std::shared_ptr< ::NAMESPACE_std_msgs::String> StringPtr;
typedef ::std::shared_ptr< ::NAMESPACE_std_msgs::String const> StringConstPtr;


} // namespace NAMESPACE_std_msgs

namespace ROSLITE_NAMESPACE
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::NAMESPACE_std_msgs::String_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.data);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct String_
} // namespace serialization
} // namespace ROSLITE_NAMESPACE

#undef NAMESPACE_std_msgs

#endif // ROSLITE_STD_MSGS_MESSAGE_STRING_H

