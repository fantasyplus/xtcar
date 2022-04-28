// Generated by gencpp from file behaviour_state_machine/GoalPoseResponse.msg
// DO NOT EDIT!


#ifndef BEHAVIOUR_STATE_MACHINE_MESSAGE_GOALPOSERESPONSE_H
#define BEHAVIOUR_STATE_MACHINE_MESSAGE_GOALPOSERESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace behaviour_state_machine
{
template <class ContainerAllocator>
struct GoalPoseResponse_
{
  typedef GoalPoseResponse_<ContainerAllocator> Type;

  GoalPoseResponse_()
    : is_success(false)  {
    }
  GoalPoseResponse_(const ContainerAllocator& _alloc)
    : is_success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _is_success_type;
  _is_success_type is_success;





  typedef boost::shared_ptr< ::behaviour_state_machine::GoalPoseResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::behaviour_state_machine::GoalPoseResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GoalPoseResponse_

typedef ::behaviour_state_machine::GoalPoseResponse_<std::allocator<void> > GoalPoseResponse;

typedef boost::shared_ptr< ::behaviour_state_machine::GoalPoseResponse > GoalPoseResponsePtr;
typedef boost::shared_ptr< ::behaviour_state_machine::GoalPoseResponse const> GoalPoseResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::behaviour_state_machine::GoalPoseResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::behaviour_state_machine::GoalPoseResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::behaviour_state_machine::GoalPoseResponse_<ContainerAllocator1> & lhs, const ::behaviour_state_machine::GoalPoseResponse_<ContainerAllocator2> & rhs)
{
  return lhs.is_success == rhs.is_success;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::behaviour_state_machine::GoalPoseResponse_<ContainerAllocator1> & lhs, const ::behaviour_state_machine::GoalPoseResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace behaviour_state_machine

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::behaviour_state_machine::GoalPoseResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::behaviour_state_machine::GoalPoseResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::behaviour_state_machine::GoalPoseResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::behaviour_state_machine::GoalPoseResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::behaviour_state_machine::GoalPoseResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::behaviour_state_machine::GoalPoseResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::behaviour_state_machine::GoalPoseResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fa3e942e5cfe76a6a46f20a0780b2cf3";
  }

  static const char* value(const ::behaviour_state_machine::GoalPoseResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xfa3e942e5cfe76a6ULL;
  static const uint64_t static_value2 = 0xa46f20a0780b2cf3ULL;
};

template<class ContainerAllocator>
struct DataType< ::behaviour_state_machine::GoalPoseResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "behaviour_state_machine/GoalPoseResponse";
  }

  static const char* value(const ::behaviour_state_machine::GoalPoseResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::behaviour_state_machine::GoalPoseResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool is_success\n"
;
  }

  static const char* value(const ::behaviour_state_machine::GoalPoseResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::behaviour_state_machine::GoalPoseResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.is_success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GoalPoseResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::behaviour_state_machine::GoalPoseResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::behaviour_state_machine::GoalPoseResponse_<ContainerAllocator>& v)
  {
    s << indent << "is_success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BEHAVIOUR_STATE_MACHINE_MESSAGE_GOALPOSERESPONSE_H