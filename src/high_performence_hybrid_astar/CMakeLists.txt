cmake_minimum_required(VERSION 3.0.2)
project(high_performence_hybrid_astar)

set(CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O2 ")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
  tf_conversions
)

## 查找OMPL库
find_package(ompl REQUIRED)

if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)

message("${OMPL_LIBRARIES}")

set(SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/high_performence_hybrid_astar.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamicvoronoi.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/bucketedqueue.cpp
)
set(HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/struct_definition.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/high_performence_hybrid_astar.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/timer.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/dynamicvoronoi.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/point.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/vector2d.h
  ${CMAKE_CURRENT_SOURCE_DIR}/include/bucketedqueue.h
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES high_performence_hybrid_astar
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/high_performence_hybrid_astar.cpp ${HEADERS} ${SOURCES})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
