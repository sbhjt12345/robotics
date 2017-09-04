// Generated by gencpp from file foundations_hw1/RewardResponse.msg
// DO NOT EDIT!


#ifndef FOUNDATIONS_HW1_MESSAGE_REWARDRESPONSE_H
#define FOUNDATIONS_HW1_MESSAGE_REWARDRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace foundations_hw1
{
template <class ContainerAllocator>
struct RewardResponse_
{
  typedef RewardResponse_<ContainerAllocator> Type;

  RewardResponse_()
    : value(0.0)  {
    }
  RewardResponse_(const ContainerAllocator& _alloc)
    : value(0.0)  {
  (void)_alloc;
    }



   typedef double _value_type;
  _value_type value;




  typedef boost::shared_ptr< ::foundations_hw1::RewardResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::foundations_hw1::RewardResponse_<ContainerAllocator> const> ConstPtr;

}; // struct RewardResponse_

typedef ::foundations_hw1::RewardResponse_<std::allocator<void> > RewardResponse;

typedef boost::shared_ptr< ::foundations_hw1::RewardResponse > RewardResponsePtr;
typedef boost::shared_ptr< ::foundations_hw1::RewardResponse const> RewardResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::foundations_hw1::RewardResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::foundations_hw1::RewardResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace foundations_hw1

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/lunar/share/geometry_msgs/cmake/../msg'], 'turtlesim': ['/opt/ros/lunar/share/turtlesim/cmake/../msg'], 'std_msgs': ['/opt/ros/lunar/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::foundations_hw1::RewardResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::foundations_hw1::RewardResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::foundations_hw1::RewardResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::foundations_hw1::RewardResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::foundations_hw1::RewardResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::foundations_hw1::RewardResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::foundations_hw1::RewardResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1b1594d2b74931ef8fe7be8e2d594455";
  }

  static const char* value(const ::foundations_hw1::RewardResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1b1594d2b74931efULL;
  static const uint64_t static_value2 = 0x8fe7be8e2d594455ULL;
};

template<class ContainerAllocator>
struct DataType< ::foundations_hw1::RewardResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "foundations_hw1/RewardResponse";
  }

  static const char* value(const ::foundations_hw1::RewardResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::foundations_hw1::RewardResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 value\n\
\n\
";
  }

  static const char* value(const ::foundations_hw1::RewardResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::foundations_hw1::RewardResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.value);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RewardResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::foundations_hw1::RewardResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::foundations_hw1::RewardResponse_<ContainerAllocator>& v)
  {
    s << indent << "value: ";
    Printer<double>::stream(s, indent + "  ", v.value);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FOUNDATIONS_HW1_MESSAGE_REWARDRESPONSE_H
