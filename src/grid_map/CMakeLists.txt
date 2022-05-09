cmake_minimum_required(VERSION 3.0.2)
project(grid_map)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  roslint
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_eigen
  tf2_ros
)

catkin_package()

set(ROSLINT_CPP_OPTS "--filter=-build/c++14")
roslint_cpp(
  src/grid_map.cpp
  src/grid_map_node.cpp
  include/grid_map.h
)

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenMP)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

link_directories(${PCL_LIBRARY_DIRS})

# add lib
add_library(grid_map_lib SHARED
  src/grid_map.cpp
)

if(OPENMP_FOUND)
  set_target_properties(grid_map_lib PROPERTIES
    COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
    LINK_FLAGS ${OpenMP_CXX_FLAGS}
  )
endif()

target_include_directories(grid_map_lib PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(grid_map_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
#   ${Qt5Core_LIBRARIES}
)

add_dependencies(grid_map_lib ${catkin_EXPORTED_TARGETS})
 
add_executable(grid_map_node
  src/grid_map_node.cpp
)
target_link_libraries(grid_map_node grid_map_lib)
add_dependencies(grid_map_node ${catkin_EXPORTED_TARGETS})


