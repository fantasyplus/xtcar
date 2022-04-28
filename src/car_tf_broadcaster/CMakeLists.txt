cmake_minimum_required(VERSION 2.8.3)
project(car_tf_broadcaster_xt)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nmea_msgs
  roscpp
  tf
  std_msgs
)

catkin_package()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
    src/car_tf_broadcaster.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} 
  ${catkin_EXPORTED_TARGETS}
)
