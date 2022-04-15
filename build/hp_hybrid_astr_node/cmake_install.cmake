# Install script for directory: /home/ros/fantasyplus/xtcar/src/hp_hybrid_astr_node

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ros/fantasyplus/xtcar/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ros/fantasyplus/xtcar/build/hp_hybrid_astr_node/catkin_generated/installspace/hp_hybrid_astr_node.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hp_hybrid_astr_node/cmake" TYPE FILE FILES
    "/home/ros/fantasyplus/xtcar/build/hp_hybrid_astr_node/catkin_generated/installspace/hp_hybrid_astr_nodeConfig.cmake"
    "/home/ros/fantasyplus/xtcar/build/hp_hybrid_astr_node/catkin_generated/installspace/hp_hybrid_astr_nodeConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hp_hybrid_astr_node" TYPE FILE FILES "/home/ros/fantasyplus/xtcar/src/hp_hybrid_astr_node/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hp_hybrid_astr_node/hp_hybrid_astr_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hp_hybrid_astr_node/hp_hybrid_astr_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hp_hybrid_astr_node/hp_hybrid_astr_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hp_hybrid_astr_node" TYPE EXECUTABLE FILES "/home/ros/fantasyplus/xtcar/devel/lib/hp_hybrid_astr_node/hp_hybrid_astr_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hp_hybrid_astr_node/hp_hybrid_astr_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hp_hybrid_astr_node/hp_hybrid_astr_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hp_hybrid_astr_node/hp_hybrid_astr_node"
         OLD_RPATH "/home/ros/autoware.ai/install/op_ros_helpers/lib:/home/ros/autoware.ai/install/libwaypoint_follower/lib:/home/ros/autoware.ai/install/amathutils_lib/lib:/home/ros/autoware.ai/install/op_simu/lib:/home/ros/autoware.ai/install/op_planner/lib:/home/ros/autoware.ai/install/op_utility/lib:/home/ros/autoware.ai/install/vector_map/lib:/opt/ros/melodic/lib:/usr/lib/x86_64-linux-gnu/hdf5/openmpi:/usr/lib/x86_64-linux-gnu/openmpi/lib:/home/ros/fantasyplus/xtcar/devel/lib:/usr/local/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hp_hybrid_astr_node/hp_hybrid_astr_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hp_hybrid_astr_node/launch" TYPE DIRECTORY FILES "/home/ros/fantasyplus/xtcar/src/hp_hybrid_astr_node/launch/")
endif()

