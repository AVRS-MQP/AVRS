// Generated by gencpp from file coms_msgs/Vehicle.msg
// DO NOT EDIT!


#ifndef COMS_MSGS_MESSAGE_VEHICLE_H
#define COMS_MSGS_MESSAGE_VEHICLE_H


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
struct Vehicle_
{
  typedef Vehicle_<ContainerAllocator> Type;

  Vehicle_()
    : header()
    , headerstamp()
    , model()
    , charger_type()
    , battery_charge(0.0)
    , charge_level(0)
    , flap_unlocked(false)
    , flap_auto_open(false)  {
    }
  Vehicle_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , headerstamp()
    , model(_alloc)
    , charger_type(_alloc)
    , battery_charge(0.0)
    , charge_level(0)
    , flap_unlocked(false)
    , flap_auto_open(false)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef ros::Time _headerstamp_type;
  _headerstamp_type headerstamp;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _model_type;
  _model_type model;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _charger_type_type;
  _charger_type_type charger_type;

   typedef float _battery_charge_type;
  _battery_charge_type battery_charge;

   typedef int32_t _charge_level_type;
  _charge_level_type charge_level;

   typedef uint8_t _flap_unlocked_type;
  _flap_unlocked_type flap_unlocked;

   typedef uint8_t _flap_auto_open_type;
  _flap_auto_open_type flap_auto_open;





  typedef boost::shared_ptr< ::coms_msgs::Vehicle_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::coms_msgs::Vehicle_<ContainerAllocator> const> ConstPtr;

}; // struct Vehicle_

typedef ::coms_msgs::Vehicle_<std::allocator<void> > Vehicle;

typedef boost::shared_ptr< ::coms_msgs::Vehicle > VehiclePtr;
typedef boost::shared_ptr< ::coms_msgs::Vehicle const> VehicleConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::coms_msgs::Vehicle_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::coms_msgs::Vehicle_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::coms_msgs::Vehicle_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::coms_msgs::Vehicle_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::coms_msgs::Vehicle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::coms_msgs::Vehicle_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::coms_msgs::Vehicle_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::coms_msgs::Vehicle_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::coms_msgs::Vehicle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "15c570864c534a992bcad74ac9993ed9";
  }

  static const char* value(const ::coms_msgs::Vehicle_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x15c570864c534a99ULL;
  static const uint64_t static_value2 = 0x2bcad74ac9993ed9ULL;
};

template<class ContainerAllocator>
struct DataType< ::coms_msgs::Vehicle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "coms_msgs/Vehicle";
  }

  static const char* value(const ::coms_msgs::Vehicle_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::coms_msgs::Vehicle_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
time headerstamp\n\
string model\n\
string charger_type\n\
float32 battery_charge\n\
int32 charge_level\n\
bool flap_unlocked\n\
bool flap_auto_open\n\
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

  static const char* value(const ::coms_msgs::Vehicle_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::coms_msgs::Vehicle_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.headerstamp);
      stream.next(m.model);
      stream.next(m.charger_type);
      stream.next(m.battery_charge);
      stream.next(m.charge_level);
      stream.next(m.flap_unlocked);
      stream.next(m.flap_auto_open);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Vehicle_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::coms_msgs::Vehicle_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::coms_msgs::Vehicle_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "headerstamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.headerstamp);
    s << indent << "model: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.model);
    s << indent << "charger_type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.charger_type);
    s << indent << "battery_charge: ";
    Printer<float>::stream(s, indent + "  ", v.battery_charge);
    s << indent << "charge_level: ";
    Printer<int32_t>::stream(s, indent + "  ", v.charge_level);
    s << indent << "flap_unlocked: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.flap_unlocked);
    s << indent << "flap_auto_open: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.flap_auto_open);
  }
};

} // namespace message_operations
} // namespace ros

#endif // COMS_MSGS_MESSAGE_VEHICLE_H
