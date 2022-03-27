cmake_minimum_required(VERSION 2.8.3)
project(hp_hybrid_astr_node)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O2")
# set (CMAKE_VERBOSE_MAKEFILE ON)

find_package(autoware_build_flags REQUIRED)
find_package(
  catkin REQUIRED COMPONENTS
    # astar_search
    autoware_msgs
    geometry_msgs
    nav_msgs
    roscpp
    tf
    visualization_msgs

    #for visualization
    op_ros_helpers
    op_simu
    op_utility
    op_planner
    vector_map_msgs
    pcl_conversions
    pcl_ros
    jsk_recognition_msgs

    #insert by xt
    high_performence_hybrid_astar
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}
  src/astar_navi/astar_navi.cpp
  src/astar_navi/astar_navi_node.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
