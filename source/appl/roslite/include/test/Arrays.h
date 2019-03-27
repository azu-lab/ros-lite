// [note] Auto-generated file
// [note] 2019-03-20T08:43:43Z
// [note] based on source/appl/ros_src/msg/test/Arrays.msg
#ifndef ROSLITE_TEST_MESSAGE_ARRAYS_H
#define ROSLITE_TEST_MESSAGE_ARRAYS_H

#include <string>
#include <vector>
#include <map>
#include <ostream>

#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/common.h"
  #include "roslite/include/ros/serialization.h"
  #include "test/Arrays.h"
#else
  #include "ros/common.h"
  #include "ros/serialization.h"
#endif

#if ROSLITE_TARGET_CLUSTER_ID == 0
  #define NAMESPACE_test roslite_test
#else
  #define NAMESPACE_test test
#endif

namespace NAMESPACE_test
{
template <class ContainerAllocator>
struct Arrays_ {
  typedef Arrays_<ContainerAllocator> Type;

  Arrays_()
  : int8_arr()
  , string_arr()
  {
  }

  Arrays_(const ContainerAllocator& _alloc)
  : int8_arr(_alloc)
  , string_arr(_alloc)
  {
  }

  typedef std::vector<int8_t, typename ContainerAllocator::template rebind<int8_t>::other >  _int8_arr_type;
  std::vector<int8_t, typename ContainerAllocator::template rebind<int8_t>::other >  int8_arr;

  typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _string_arr_type;
  std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  string_arr;


  typedef ::std::shared_ptr< ::NAMESPACE_test::Arrays_<ContainerAllocator> > Ptr;
  typedef ::std::shared_ptr< ::NAMESPACE_test::Arrays_<ContainerAllocator>  const> ConstPtr;

#if ROSLITE_TARGET_CLUSTER_ID == 0
  static Type FromRosMsg(const typename test::Arrays_<ContainerAllocator>& ros_msg) {
    Type roslite_msg;
    roslite_msg.int8_arr = ros_msg.int8_arr;
    roslite_msg.string_arr = ros_msg.string_arr;
    return roslite_msg;
  }

  typename test::Arrays_<ContainerAllocator> ToRosMsg() const {
    typename test::Arrays_<ContainerAllocator> ros_msg;
    ros_msg.int8_arr = int8_arr;
    ros_msg.string_arr = string_arr;
    return ros_msg;
  }
  
  static typename Type::ConstPtr FromRosMsgPtr(const typename test::Arrays_<ContainerAllocator>::ConstPtr& ros_msg) {
    typename Type::Ptr roslite_msg(new Type());
    *roslite_msg = FromRosMsg(*ros_msg);
    return roslite_msg;
  }

  typename test::Arrays_<ContainerAllocator>::ConstPtr ToRosMsgPtr() const {
    typename test::Arrays_<ContainerAllocator>::Ptr ros_msg(new test::Arrays_<ContainerAllocator>());
    *ros_msg = ToRosMsg();
    return ros_msg;
  }
#endif

}; // struct Arrays
typedef  ::NAMESPACE_test::Arrays_<std::allocator<void> > Arrays;

typedef ::std::shared_ptr< ::NAMESPACE_test::Arrays> ArraysPtr;
typedef ::std::shared_ptr< ::NAMESPACE_test::Arrays const> ArraysConstPtr;


} // namespace NAMESPACE_test

namespace ROSLITE_NAMESPACE
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::NAMESPACE_test::Arrays_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.int8_arr);
    stream.next(m.string_arr);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Arrays_
} // namespace serialization
} // namespace ROSLITE_NAMESPACE

#undef NAMESPACE_test

#endif // ROSLITE_TEST_MESSAGE_ARRAYS_H

