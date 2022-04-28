// Generated by gencpp from file behaviour_state_machine/GoalPoseRequest.msg
// DO NOT EDIT!


#ifndef BEHAVIOUR_STATE_MACHINE_MESSAGE_GOALPOSEREQUEST_H
#define BEHAVIOUR_STATE_MACHINE_MESSAGE_GOALPOSEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>

namespace behaviour_state_machine
{
template <class ContainerAllocator>
struct GoalPoseRequest_
{
  typedef GoalPoseRequest_<ContainerAllocator> Type;

  GoalPoseRequest_()
    : header()
    , pose()  {
    }
  GoalPoseRequest_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , pose(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;





  typedef boost::shared_ptr< ::behaviour_state_machine::GoalPoseRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::behaviour_state_machine::GoalPoseRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GoalPoseRequest_

typedef ::behaviour_state_machine::GoalPoseRequest_<std::allocator<void> > GoalPoseRequest;

typedef boost::shared_ptr< ::behaviour_state_machine::GoalPoseRequest > GoalPoseRequestPtr;
typedef boost::shared_ptr< ::behaviour_state_machine::GoalPoseRequest const> GoalPoseRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::behaviour_state_machine::GoalPoseRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::behaviour_state_machine::GoalPoseRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::behaviour_state_machine::GoalPoseRequest_<ContainerAllocator1> & lhs, const ::behaviour_state_machine::GoalPoseRequest_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.pose == rhs.pose;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::behaviour_state_machine::GoalPoseRequest_<ContainerAllocator1> & lhs, const ::behaviour_state_machine::GoalPoseRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace behaviour_state_machine

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::behaviour_state_machine::GoalPoseRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::behaviour_state_machine::GoalPoseRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::behaviour_state_machine::GoalPoseRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::behaviour_state_machine::GoalPoseRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::behaviour_state_machine::GoalPoseRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::behaviour_state_machine::GoalPoseRequest_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::behaviour_state_machine::GoalPoseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d3812c3cbc69362b77dc0b19b345f8f5";
  }

  static const char* value(const ::behaviour_state_machine::GoalPoseRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd3812c3cbc69362bULL;
  static const uint64_t static_value2 = 0x77dc0b19b345f8f5ULL;
};

template<class ContainerAllocator>
struct DataType< ::behaviour_state_machine::GoalPoseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "behaviour_state_machine/GoalPoseRequest";
  }

  static const char* value(const ::behaviour_state_machine::GoalPoseRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::behaviour_state_machine::GoalPoseRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"geometry_msgs/Pose pose\n"
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
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::behaviour_state_machine::GoalPoseRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::behaviour_state_machine::GoalPoseRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.pose);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GoalPoseRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::behaviour_state_machine::GoalPoseRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::behaviour_state_machine::GoalPoseRequest_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BEHAVIOUR_STATE_MACHINE_MESSAGE_GOALPOSEREQUEST_H
