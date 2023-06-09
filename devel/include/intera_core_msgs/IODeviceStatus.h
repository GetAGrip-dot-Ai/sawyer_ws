// Generated by gencpp from file intera_core_msgs/IODeviceStatus.msg
// DO NOT EDIT!


#ifndef INTERA_CORE_MSGS_MESSAGE_IODEVICESTATUS_H
#define INTERA_CORE_MSGS_MESSAGE_IODEVICESTATUS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <intera_core_msgs/IOComponentStatus.h>
#include <intera_core_msgs/IODataStatus.h>
#include <intera_core_msgs/IODataStatus.h>

namespace intera_core_msgs
{
template <class ContainerAllocator>
struct IODeviceStatus_
{
  typedef IODeviceStatus_<ContainerAllocator> Type;

  IODeviceStatus_()
    : time()
    , device()
    , ports()
    , signals()
    , commands()
    , responses()  {
    }
  IODeviceStatus_(const ContainerAllocator& _alloc)
    : time()
    , device(_alloc)
    , ports(_alloc)
    , signals(_alloc)
    , commands(_alloc)
    , responses(_alloc)  {
  (void)_alloc;
    }



   typedef ros::Time _time_type;
  _time_type time;

   typedef  ::intera_core_msgs::IOComponentStatus_<ContainerAllocator>  _device_type;
  _device_type device;

   typedef std::vector< ::intera_core_msgs::IODataStatus_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::intera_core_msgs::IODataStatus_<ContainerAllocator> >> _ports_type;
  _ports_type ports;

   typedef std::vector< ::intera_core_msgs::IODataStatus_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::intera_core_msgs::IODataStatus_<ContainerAllocator> >> _signals_type;
  _signals_type signals;

   typedef std::vector<ros::Time, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros::Time>> _commands_type;
  _commands_type commands;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> _responses_type;
  _responses_type responses;





  typedef boost::shared_ptr< ::intera_core_msgs::IODeviceStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::intera_core_msgs::IODeviceStatus_<ContainerAllocator> const> ConstPtr;

}; // struct IODeviceStatus_

typedef ::intera_core_msgs::IODeviceStatus_<std::allocator<void> > IODeviceStatus;

typedef boost::shared_ptr< ::intera_core_msgs::IODeviceStatus > IODeviceStatusPtr;
typedef boost::shared_ptr< ::intera_core_msgs::IODeviceStatus const> IODeviceStatusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::intera_core_msgs::IODeviceStatus_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::intera_core_msgs::IODeviceStatus_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::intera_core_msgs::IODeviceStatus_<ContainerAllocator1> & lhs, const ::intera_core_msgs::IODeviceStatus_<ContainerAllocator2> & rhs)
{
  return lhs.time == rhs.time &&
    lhs.device == rhs.device &&
    lhs.ports == rhs.ports &&
    lhs.signals == rhs.signals &&
    lhs.commands == rhs.commands &&
    lhs.responses == rhs.responses;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::intera_core_msgs::IODeviceStatus_<ContainerAllocator1> & lhs, const ::intera_core_msgs::IODeviceStatus_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace intera_core_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::intera_core_msgs::IODeviceStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_core_msgs::IODeviceStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::intera_core_msgs::IODeviceStatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::intera_core_msgs::IODeviceStatus_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_core_msgs::IODeviceStatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_core_msgs::IODeviceStatus_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::intera_core_msgs::IODeviceStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0d0c97a2d700848e7ad46e09a77cf896";
  }

  static const char* value(const ::intera_core_msgs::IODeviceStatus_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0d0c97a2d700848eULL;
  static const uint64_t static_value2 = 0x7ad46e09a77cf896ULL;
};

template<class ContainerAllocator>
struct DataType< ::intera_core_msgs::IODeviceStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "intera_core_msgs/IODeviceStatus";
  }

  static const char* value(const ::intera_core_msgs::IODeviceStatus_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::intera_core_msgs::IODeviceStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "## IO Device status\n"
"time time                  # time the message was created\n"
"IOComponentStatus device   # device status data\n"
"IODataStatus[]    ports    # Ports status\n"
"IODataStatus[]    signals  # Signals status\n"
"time[]            commands # recent command timestamps, for syncing\n"
"string[]          responses # recent command responses\n"
"\n"
"\n"
"================================================================================\n"
"MSG: intera_core_msgs/IOComponentStatus\n"
"## IO Component status data\n"
"string name            # component name\n"
"IOStatus status        # component status\n"
"#\n"
"\n"
"\n"
"\n"
"================================================================================\n"
"MSG: intera_core_msgs/IOStatus\n"
"## IO status data\n"
"#\n"
"string tag             # one of the values listed below\n"
"#   down     Inoperative, not fully instantiated\n"
"#   ready    OK, fully operational\n"
"#   busy     OK, not ready to output data; input data value may be stale\n"
"#   unready  OK, not operational; data is invalid\n"
"#   error    Error, not operational\n"
"string DOWN      = down\n"
"string READY     = ready\n"
"string BUSY      = busy\n"
"string UNREADY   = unready\n"
"string ERROR     = error\n"
"#\n"
"string id             # message id, for internationalization\n"
"#\n"
"string detail         # optional additional status detail\n"
"#\n"
"\n"
"================================================================================\n"
"MSG: intera_core_msgs/IODataStatus\n"
"## IO Data Status\n"
"string name       # IO Data Component name\n"
"string format     # data format:\n"
"                  # A JSON object containing one or more of the following fields:\n"
"                  # Required:\n"
"                  #   \"type\" : \"<type>\"  JSON Type, one of:\n"
"                  #                      \"bool\", \"int\", \"float\", \"string\", \"object\", \"array\"\n"
"                  #\n"
"                  # Optional:\n"
"                  #   \"dimensions\" : [ N,...] if absent, [ 1 ] is assumed.\n"
"                  #   \"role\"       : Signal: \"input\", \"output\"; Port:  \"sink\", \"source\"\n"
"                  #   \"data_type\"  : qualifier for the JSON type, may be device-specific\n"
"                  #                  int data_types:  \"uN\", \"sN\", \"NqM\"\n"
"                  #   other device-specific qualifiers may be used and will be passed through.\n"
"string data       # JSON data value.  An Array is expected, for example \"[ 0 ]\"\n"
"IOStatus status   # Data Component status\n"
;
  }

  static const char* value(const ::intera_core_msgs::IODeviceStatus_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::intera_core_msgs::IODeviceStatus_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.time);
      stream.next(m.device);
      stream.next(m.ports);
      stream.next(m.signals);
      stream.next(m.commands);
      stream.next(m.responses);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct IODeviceStatus_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::intera_core_msgs::IODeviceStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::intera_core_msgs::IODeviceStatus_<ContainerAllocator>& v)
  {
    s << indent << "time: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.time);
    s << indent << "device: ";
    s << std::endl;
    Printer< ::intera_core_msgs::IOComponentStatus_<ContainerAllocator> >::stream(s, indent + "  ", v.device);
    s << indent << "ports[]" << std::endl;
    for (size_t i = 0; i < v.ports.size(); ++i)
    {
      s << indent << "  ports[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::intera_core_msgs::IODataStatus_<ContainerAllocator> >::stream(s, indent + "    ", v.ports[i]);
    }
    s << indent << "signals[]" << std::endl;
    for (size_t i = 0; i < v.signals.size(); ++i)
    {
      s << indent << "  signals[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::intera_core_msgs::IODataStatus_<ContainerAllocator> >::stream(s, indent + "    ", v.signals[i]);
    }
    s << indent << "commands[]" << std::endl;
    for (size_t i = 0; i < v.commands.size(); ++i)
    {
      s << indent << "  commands[" << i << "]: ";
      Printer<ros::Time>::stream(s, indent + "  ", v.commands[i]);
    }
    s << indent << "responses[]" << std::endl;
    for (size_t i = 0; i < v.responses.size(); ++i)
    {
      s << indent << "  responses[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.responses[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // INTERA_CORE_MSGS_MESSAGE_IODEVICESTATUS_H
