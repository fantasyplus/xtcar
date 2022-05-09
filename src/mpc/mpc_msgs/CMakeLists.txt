cmake_minimum_required(VERSION 3.0.2)
project(mpc_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  jsk_recognition_msgs
)


add_message_files(
  FILES
  Waypoint.msg
  Lane.msg
  VehicleStatus.msg
  ControlCommand.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  jsk_recognition_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mpc_msgs
   CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
