// Generated by gencpp from file coms_msgs/Station.msg
// DO NOT EDIT!


#ifndef COMS_MSGS_MESSAGE_STATION_H
#define COMS_MSGS_MESSAGE_STATION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace coms_msgs
{
template <class ContainerAllocator>
struct Station_
{
  typedef Station_<ContainerAllocator> Type;

  Station_()
    : header()
    , headerstamp()
    , xMax(0.0)
    , xMin(0.0)
    , yMax(0.0)
    , yMin(0.0)
    , zMax(0.0)
    , zMin(0.0)
    , flap_not_open_error(false)  {
    }
  Station_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , headerstamp()
    , xMax(0.0)
    , xMin(0.0)
    , yMax(0.0)
    , yMin(0.0)
    , zMax(0.0)
    , zMin(0.0)
    , flap_not_open_error(false)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef ros::Time _headerstamp_type;
  _headerstamp_type headerstamp;

   typedef float _xMax_type;
  _xMax_type xMax;

   typedef float _xMin_type;
  _xMin_type xMin;

   typedef float _yMax_type;
  _yMax_type yMax;

   typedef float _yMin_type;
  _yMin_type yMin;

   typedef float _zMax_type;
  _zMax_type zMax;

   typedef float _zMin_type;
  _zMin_type zMin;

   typedef uint8_t _flap_not_open_error_type;
  _flap_not_open_error_type flap_not_open_error;





  typedef boost::shared_ptr< ::coms_msgs::Station_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::coms_msgs::Station_<ContainerAllocator> const> ConstPtr;

}; // struct Station_

typedef ::coms_msgs::Station_<std::allocator<void> > Station;

typedef boost::shared_ptr< ::coms_msgs::Station > StationPtr;
typedef boost::shared_ptr< ::coms_msgs::Station const> StationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::coms_msgs::Station_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::coms_msgs::Station_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace coms_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'coms_msgs': ['/home/avrs/AVRS_ws/src/msgs/coms_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::coms_msgs::Station_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::coms_msgs::Station_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::coms_msgs::Station_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::coms_msgs::Station_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::coms_msgs::Station_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::coms_msgs::Station_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::coms_msgs::Station_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c279ad04cbc81207a5714883b84da036";
  }

  static const char* value(const ::coms_msgs::Station_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc279ad04cbc81207ULL;
  static const uint64_t static_value2 = 0xa5714883b84da036ULL;
};

template<class ContainerAllocator>
struct DataType< ::coms_msgs::Station_<ContainerAllocator> >
{
  static const char* value()
  {
    return "coms_msgs/Station";
  }

  static const char* value(const ::coms_msgs::Station_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::coms_msgs::Station_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
time headerstamp\n\
float32 xMax\n\
float32 xMin\n\
float32 yMax\n\
float32 yMin\n\
float32 zMax\n\
float32 zMin\n\
bool flap_not_open_error\n\
\n\
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

  static const char* value(const ::coms_msgs::Station_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::coms_msgs::Station_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.headerstamp);
      stream.next(m.xMax);
      stream.next(m.xMin);
      stream.next(m.yMax);
      stream.next(m.yMin);
      stream.next(m.zMax);
      stream.next(m.zMin);
      stream.next(m.flap_not_open_error);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Station_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::coms_msgs::Station_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::coms_msgs::Station_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "headerstamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.headerstamp);
    s << indent << "xMax: ";
    Printer<float>::stream(s, indent + "  ", v.xMax);
    s << indent << "xMin: ";
    Printer<float>::stream(s, indent + "  ", v.xMin);
    s << indent << "yMax: ";
    Printer<float>::stream(s, indent + "  ", v.yMax);
    s << indent << "yMin: ";
    Printer<float>::stream(s, indent + "  ", v.yMin);
    s << indent << "zMax: ";
    Printer<float>::stream(s, indent + "  ", v.zMax);
    s << indent << "zMin: ";
    Printer<float>::stream(s, indent + "  ", v.zMin);
    s << indent << "flap_not_open_error: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.flap_not_open_error);
  }
};

} // namespace message_operations
} // namespace ros

#endif // COMS_MSGS_MESSAGE_STATION_H
