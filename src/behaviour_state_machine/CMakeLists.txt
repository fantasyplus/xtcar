cmake_minimum_required(VERSION 2.8.3)
project(behaviour_state_machine)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  tf
  tf2_msgs
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  std_msgs
  message_generation
)

#添加srv文件相关
add_service_files(
  FILES
  GoalPose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs mpc_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

#生成可执行文件
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
    src/behaviour_state_machine_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} 
  ${catkin_EXPORTED_TARGETS}
)
