cmake_minimum_required(VERSION 2.8.3)
project(car_simulator)

find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  amathutils_lib
  mpc_msgs
  geometry_msgs
  roscpp
  roslint
  std_msgs
  tf
  tf2
  vehicle_sim_model
)

catkin_package(
  INCLUDE_DIRS include
)

set(ROSLINT_CPP_OPTS "--filter=-build/c++11")
roslint_cpp()

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/car_simulator_node.cpp
  src/car_simulator_core.cpp
  src/vehicle_model_ros.cpp
)

target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
