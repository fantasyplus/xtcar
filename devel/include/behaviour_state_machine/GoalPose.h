// Generated by gencpp from file behaviour_state_machine/GoalPose.msg
// DO NOT EDIT!


#ifndef BEHAVIOUR_STATE_MACHINE_MESSAGE_GOALPOSE_H
#define BEHAVIOUR_STATE_MACHINE_MESSAGE_GOALPOSE_H

#include <ros/service_traits.h>


#include <behaviour_state_machine/GoalPoseRequest.h>
#include <behaviour_state_machine/GoalPoseResponse.h>


namespace behaviour_state_machine
{

struct GoalPose
{

typedef GoalPoseRequest Request;
typedef GoalPoseResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GoalPose
} // namespace behaviour_state_machine


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::behaviour_state_machine::GoalPose > {
  static const char* value()
  {
    return "0aad1829ef886d046d8951a77cf3842c";
  }

  static const char* value(const ::behaviour_state_machine::GoalPose&) { return value(); }
};

template<>
struct DataType< ::behaviour_state_machine::GoalPose > {
  static const char* value()
  {
    return "behaviour_state_machine/GoalPose";
  }

  static const char* value(const ::behaviour_state_machine::GoalPose&) { return value(); }
};


// service_traits::MD5Sum< ::behaviour_state_machine::GoalPoseRequest> should match
// service_traits::MD5Sum< ::behaviour_state_machine::GoalPose >
template<>
struct MD5Sum< ::behaviour_state_machine::GoalPoseRequest>
{
  static const char* value()
  {
    return MD5Sum< ::behaviour_state_machine::GoalPose >::value();
  }
  static const char* value(const ::behaviour_state_machine::GoalPoseRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::behaviour_state_machine::GoalPoseRequest> should match
// service_traits::DataType< ::behaviour_state_machine::GoalPose >
template<>
struct DataType< ::behaviour_state_machine::GoalPoseRequest>
{
  static const char* value()
  {
    return DataType< ::behaviour_state_machine::GoalPose >::value();
  }
  static const char* value(const ::behaviour_state_machine::GoalPoseRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::behaviour_state_machine::GoalPoseResponse> should match
// service_traits::MD5Sum< ::behaviour_state_machine::GoalPose >
template<>
struct MD5Sum< ::behaviour_state_machine::GoalPoseResponse>
{
  static const char* value()
  {
    return MD5Sum< ::behaviour_state_machine::GoalPose >::value();
  }
  static const char* value(const ::behaviour_state_machine::GoalPoseResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::behaviour_state_machine::GoalPoseResponse> should match
// service_traits::DataType< ::behaviour_state_machine::GoalPose >
template<>
struct DataType< ::behaviour_state_machine::GoalPoseResponse>
{
  static const char* value()
  {
    return DataType< ::behaviour_state_machine::GoalPose >::value();
  }
  static const char* value(const ::behaviour_state_machine::GoalPoseResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // BEHAVIOUR_STATE_MACHINE_MESSAGE_GOALPOSE_H