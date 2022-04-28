cmake_minimum_required(VERSION 2.8.3)
project(hybrid_astar_node)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O2")

find_package(
  catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    roscpp
    tf
    visualization_msgs
    pcl_ros
    #insert by xt
    high_performence_hybrid_astar
    behaviour_state_machine
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/hybrid_astar_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)