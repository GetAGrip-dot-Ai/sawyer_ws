// Generated by gencpp from file intera_motion_msgs/MotionCommandFeedback.msg
// DO NOT EDIT!


#ifndef INTERA_MOTION_MSGS_MESSAGE_MOTIONCOMMANDFEEDBACK_H
#define INTERA_MOTION_MSGS_MESSAGE_MOTIONCOMMANDFEEDBACK_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <intera_motion_msgs/MotionStatus.h>

namespace intera_motion_msgs
{
template <class ContainerAllocator>
struct MotionCommandFeedback_
{
  typedef MotionCommandFeedback_<ContainerAllocator> Type;

  MotionCommandFeedback_()
    : status()  {
    }
  MotionCommandFeedback_(const ContainerAllocator& _alloc)
    : status(_alloc)  {
  (void)_alloc;
    }



   typedef  ::intera_motion_msgs::MotionStatus_<ContainerAllocator>  _status_type;
  _status_type status;





  typedef boost::shared_ptr< ::intera_motion_msgs::MotionCommandFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::intera_motion_msgs::MotionCommandFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct MotionCommandFeedback_

typedef ::intera_motion_msgs::MotionCommandFeedback_<std::allocator<void> > MotionCommandFeedback;

typedef boost::shared_ptr< ::intera_motion_msgs::MotionCommandFeedback > MotionCommandFeedbackPtr;
typedef boost::shared_ptr< ::intera_motion_msgs::MotionCommandFeedback const> MotionCommandFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::intera_motion_msgs::MotionCommandFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::intera_motion_msgs::MotionCommandFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::intera_motion_msgs::MotionCommandFeedback_<ContainerAllocator1> & lhs, const ::intera_motion_msgs::MotionCommandFeedback_<ContainerAllocator2> & rhs)
{
  return lhs.status == rhs.status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::intera_motion_msgs::MotionCommandFeedback_<ContainerAllocator1> & lhs, const ::intera_motion_msgs::MotionCommandFeedback_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace intera_motion_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::intera_motion_msgs::MotionCommandFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_motion_msgs::MotionCommandFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::intera_motion_msgs::MotionCommandFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::intera_motion_msgs::MotionCommandFeedback_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_motion_msgs::MotionCommandFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_motion_msgs::MotionCommandFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::intera_motion_msgs::MotionCommandFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "896254458c1272a4b412118474588e67";
  }

  static const char* value(const ::intera_motion_msgs::MotionCommandFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x896254458c1272a4ULL;
  static const uint64_t static_value2 = 0xb412118474588e67ULL;
};

template<class ContainerAllocator>
struct DataType< ::intera_motion_msgs::MotionCommandFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "intera_motion_msgs/MotionCommandFeedback";
  }

  static const char* value(const ::intera_motion_msgs::MotionCommandFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::intera_motion_msgs::MotionCommandFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"# feedback\n"
"MotionStatus status\n"
"\n"
"\n"
"================================================================================\n"
"MSG: intera_motion_msgs/MotionStatus\n"
"# motion status\n"
"Header header\n"
"string motion_status\n"
"string current_trajectory\n"
"uint32 current_waypoint\n"
"uint32 motion_request\n"
"\n"
"# motion_status enum values:\n"
"string MOTION_IDLE=idle\n"
"string MOTION_PENDING=pending\n"
"string MOTION_RUNNING=running\n"
"string MOTION_STOPPING=stopping\n"
"string MOTION_DONE=done\n"
"string MOTION_PREEMPTED=preempted\n"
"string MOTION_ERROR=error\n"
"\n"
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

  static const char* value(const ::intera_motion_msgs::MotionCommandFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::intera_motion_msgs::MotionCommandFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MotionCommandFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::intera_motion_msgs::MotionCommandFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::intera_motion_msgs::MotionCommandFeedback_<ContainerAllocator>& v)
  {
    s << indent << "status: ";
    s << std::endl;
    Printer< ::intera_motion_msgs::MotionStatus_<ContainerAllocator> >::stream(s, indent + "  ", v.status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // INTERA_MOTION_MSGS_MESSAGE_MOTIONCOMMANDFEEDBACK_H
