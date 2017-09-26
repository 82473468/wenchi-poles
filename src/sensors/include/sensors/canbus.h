// Generated by gencpp from file sensors/canbus.msg
// DO NOT EDIT!


#ifndef SENSORS_MESSAGE_CANBUS_H
#define SENSORS_MESSAGE_CANBUS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace sensors
{
template <class ContainerAllocator>
struct canbus_
{
  typedef canbus_<ContainerAllocator> Type;

  canbus_()
    : header()
    , speed(0.0)
    , steering(0.0)  {
    }
  canbus_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , speed(0.0)
    , steering(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _speed_type;
  _speed_type speed;

   typedef float _steering_type;
  _steering_type steering;




  typedef boost::shared_ptr< ::sensors::canbus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sensors::canbus_<ContainerAllocator> const> ConstPtr;

}; // struct canbus_

typedef ::sensors::canbus_<std::allocator<void> > canbus;

typedef boost::shared_ptr< ::sensors::canbus > canbusPtr;
typedef boost::shared_ptr< ::sensors::canbus const> canbusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sensors::canbus_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sensors::canbus_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace sensors

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'sensors': ['/home/galaxy/Pictures/rs_pro/src/RS_Localization/sensors/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::sensors::canbus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sensors::canbus_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sensors::canbus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sensors::canbus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sensors::canbus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sensors::canbus_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sensors::canbus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d9eb3de236a1305159fd802fe68cb43d";
  }

  static const char* value(const ::sensors::canbus_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd9eb3de236a13051ULL;
  static const uint64_t static_value2 = 0x59fd802fe68cb43dULL;
};

template<class ContainerAllocator>
struct DataType< ::sensors::canbus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sensors/canbus";
  }

  static const char* value(const ::sensors::canbus_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sensors::canbus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
float32 speed\n\
float32 steering\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::sensors::canbus_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sensors::canbus_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.speed);
      stream.next(m.steering);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct canbus_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sensors::canbus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sensors::canbus_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "speed: ";
    Printer<float>::stream(s, indent + "  ", v.speed);
    s << indent << "steering: ";
    Printer<float>::stream(s, indent + "  ", v.steering);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SENSORS_MESSAGE_CANBUS_H
