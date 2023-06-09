// Generated by gencpp from file intera_core_msgs/InteractionControlState.msg
// DO NOT EDIT!


#ifndef INTERA_CORE_MSGS_MESSAGE_INTERACTIONCONTROLSTATE_H
#define INTERA_CORE_MSGS_MESSAGE_INTERACTIONCONTROLSTATE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace intera_core_msgs
{
template <class ContainerAllocator>
struct InteractionControlState_
{
  typedef InteractionControlState_<ContainerAllocator> Type;

  InteractionControlState_()
    : header()
    , interaction_control_active(false)
    , K_impedance()
    , D_impedance()
    , endpoint_force_command()
    , endpoint_name()
    , in_endpoint_frame(false)
    , disable_damping_in_force_control(false)
    , disable_reference_resetting(false)
    , rotations_for_constrained_zeroG(false)  {
    }
  InteractionControlState_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , interaction_control_active(false)
    , K_impedance(_alloc)
    , D_impedance(_alloc)
    , endpoint_force_command(_alloc)
    , endpoint_name(_alloc)
    , in_endpoint_frame(false)
    , disable_damping_in_force_control(false)
    , disable_reference_resetting(false)
    , rotations_for_constrained_zeroG(false)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _interaction_control_active_type;
  _interaction_control_active_type interaction_control_active;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _K_impedance_type;
  _K_impedance_type K_impedance;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _D_impedance_type;
  _D_impedance_type D_impedance;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _endpoint_force_command_type;
  _endpoint_force_command_type endpoint_force_command;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _endpoint_name_type;
  _endpoint_name_type endpoint_name;

   typedef uint8_t _in_endpoint_frame_type;
  _in_endpoint_frame_type in_endpoint_frame;

   typedef uint8_t _disable_damping_in_force_control_type;
  _disable_damping_in_force_control_type disable_damping_in_force_control;

   typedef uint8_t _disable_reference_resetting_type;
  _disable_reference_resetting_type disable_reference_resetting;

   typedef uint8_t _rotations_for_constrained_zeroG_type;
  _rotations_for_constrained_zeroG_type rotations_for_constrained_zeroG;





  typedef boost::shared_ptr< ::intera_core_msgs::InteractionControlState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::intera_core_msgs::InteractionControlState_<ContainerAllocator> const> ConstPtr;

}; // struct InteractionControlState_

typedef ::intera_core_msgs::InteractionControlState_<std::allocator<void> > InteractionControlState;

typedef boost::shared_ptr< ::intera_core_msgs::InteractionControlState > InteractionControlStatePtr;
typedef boost::shared_ptr< ::intera_core_msgs::InteractionControlState const> InteractionControlStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::intera_core_msgs::InteractionControlState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::intera_core_msgs::InteractionControlState_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::intera_core_msgs::InteractionControlState_<ContainerAllocator1> & lhs, const ::intera_core_msgs::InteractionControlState_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.interaction_control_active == rhs.interaction_control_active &&
    lhs.K_impedance == rhs.K_impedance &&
    lhs.D_impedance == rhs.D_impedance &&
    lhs.endpoint_force_command == rhs.endpoint_force_command &&
    lhs.endpoint_name == rhs.endpoint_name &&
    lhs.in_endpoint_frame == rhs.in_endpoint_frame &&
    lhs.disable_damping_in_force_control == rhs.disable_damping_in_force_control &&
    lhs.disable_reference_resetting == rhs.disable_reference_resetting &&
    lhs.rotations_for_constrained_zeroG == rhs.rotations_for_constrained_zeroG;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::intera_core_msgs::InteractionControlState_<ContainerAllocator1> & lhs, const ::intera_core_msgs::InteractionControlState_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace intera_core_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::intera_core_msgs::InteractionControlState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_core_msgs::InteractionControlState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::intera_core_msgs::InteractionControlState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::intera_core_msgs::InteractionControlState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_core_msgs::InteractionControlState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_core_msgs::InteractionControlState_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::intera_core_msgs::InteractionControlState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f3fbd4a2356cb48da2df759db65614d8";
  }

  static const char* value(const ::intera_core_msgs::InteractionControlState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf3fbd4a2356cb48dULL;
  static const uint64_t static_value2 = 0xa2df759db65614d8ULL;
};

template<class ContainerAllocator>
struct DataType< ::intera_core_msgs::InteractionControlState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "intera_core_msgs/InteractionControlState";
  }

  static const char* value(const ::intera_core_msgs::InteractionControlState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::intera_core_msgs::InteractionControlState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "## InteractionControlState.msg ##\n"
"# Internal state of the interaction controller including\n"
"# whether the controller is active, current impedence parameters,\n"
"# and the commanded endpoint forces by the interaction controller\n"
"\n"
"Header header\n"
"\n"
"bool      interaction_control_active\n"
"\n"
"## Impedance Control Parameters\n"
"float64[] K_impedance\n"
"float64[] D_impedance\n"
"\n"
"## Force Control Parameters\n"
"# Vector of forces (wrench) (N and Nm) commanded by the interaction controller\n"
"# for the endpoint.\n"
"float64[] endpoint_force_command\n"
"\n"
"string endpoint_name\n"
"bool in_endpoint_frame\n"
"bool disable_damping_in_force_control\n"
"bool disable_reference_resetting\n"
"\n"
"## Parameters for Constrained Zero-G Behaviors\n"
"# Please refer to InteractionControlCommand.msg for more details\n"
"bool rotations_for_constrained_zeroG\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::intera_core_msgs::InteractionControlState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::intera_core_msgs::InteractionControlState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.interaction_control_active);
      stream.next(m.K_impedance);
      stream.next(m.D_impedance);
      stream.next(m.endpoint_force_command);
      stream.next(m.endpoint_name);
      stream.next(m.in_endpoint_frame);
      stream.next(m.disable_damping_in_force_control);
      stream.next(m.disable_reference_resetting);
      stream.next(m.rotations_for_constrained_zeroG);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct InteractionControlState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::intera_core_msgs::InteractionControlState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::intera_core_msgs::InteractionControlState_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "interaction_control_active: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.interaction_control_active);
    s << indent << "K_impedance[]" << std::endl;
    for (size_t i = 0; i < v.K_impedance.size(); ++i)
    {
      s << indent << "  K_impedance[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.K_impedance[i]);
    }
    s << indent << "D_impedance[]" << std::endl;
    for (size_t i = 0; i < v.D_impedance.size(); ++i)
    {
      s << indent << "  D_impedance[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.D_impedance[i]);
    }
    s << indent << "endpoint_force_command[]" << std::endl;
    for (size_t i = 0; i < v.endpoint_force_command.size(); ++i)
    {
      s << indent << "  endpoint_force_command[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.endpoint_force_command[i]);
    }
    s << indent << "endpoint_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.endpoint_name);
    s << indent << "in_endpoint_frame: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.in_endpoint_frame);
    s << indent << "disable_damping_in_force_control: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.disable_damping_in_force_control);
    s << indent << "disable_reference_resetting: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.disable_reference_resetting);
    s << indent << "rotations_for_constrained_zeroG: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.rotations_for_constrained_zeroG);
  }
};

} // namespace message_operations
} // namespace ros

#endif // INTERA_CORE_MSGS_MESSAGE_INTERACTIONCONTROLSTATE_H
