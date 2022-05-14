# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "behaviour_state_machine: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Impc_msgs:/home/ros/fantasyplus/xtcar/src/mpc/mpc_msgs/msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(behaviour_state_machine_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ros/fantasyplus/xtcar/src/behaviour_state_machine/srv/GoalPose.srv" NAME_WE)
add_custom_target(_behaviour_state_machine_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "behaviour_state_machine" "/home/ros/fantasyplus/xtcar/src/behaviour_state_machine/srv/GoalPose.srv" "mpc_msgs/Waypoint:geometry_msgs/TwistStamped:geometry_msgs/Twist:geometry_msgs/Vector3:geometry_msgs/Pose:mpc_msgs/Lane:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/PoseStamped:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(behaviour_state_machine
  "/home/ros/fantasyplus/xtcar/src/behaviour_state_machine/srv/GoalPose.srv"
  "${MSG_I_FLAGS}"
  "/home/ros/fantasyplus/xtcar/src/mpc/mpc_msgs/msg/Waypoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/ros/fantasyplus/xtcar/src/mpc/mpc_msgs/msg/Lane.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/behaviour_state_machine
)

### Generating Module File
_generate_module_cpp(behaviour_state_machine
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/behaviour_state_machine
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(behaviour_state_machine_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(behaviour_state_machine_generate_messages behaviour_state_machine_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros/fantasyplus/xtcar/src/behaviour_state_machine/srv/GoalPose.srv" NAME_WE)
add_dependencies(behaviour_state_machine_generate_messages_cpp _behaviour_state_machine_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(behaviour_state_machine_gencpp)
add_dependencies(behaviour_state_machine_gencpp behaviour_state_machine_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS behaviour_state_machine_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(behaviour_state_machine
  "/home/ros/fantasyplus/xtcar/src/behaviour_state_machine/srv/GoalPose.srv"
  "${MSG_I_FLAGS}"
  "/home/ros/fantasyplus/xtcar/src/mpc/mpc_msgs/msg/Waypoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/ros/fantasyplus/xtcar/src/mpc/mpc_msgs/msg/Lane.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/behaviour_state_machine
)

### Generating Module File
_generate_module_eus(behaviour_state_machine
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/behaviour_state_machine
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(behaviour_state_machine_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(behaviour_state_machine_generate_messages behaviour_state_machine_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros/fantasyplus/xtcar/src/behaviour_state_machine/srv/GoalPose.srv" NAME_WE)
add_dependencies(behaviour_state_machine_generate_messages_eus _behaviour_state_machine_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(behaviour_state_machine_geneus)
add_dependencies(behaviour_state_machine_geneus behaviour_state_machine_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS behaviour_state_machine_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(behaviour_state_machine
  "/home/ros/fantasyplus/xtcar/src/behaviour_state_machine/srv/GoalPose.srv"
  "${MSG_I_FLAGS}"
  "/home/ros/fantasyplus/xtcar/src/mpc/mpc_msgs/msg/Waypoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/ros/fantasyplus/xtcar/src/mpc/mpc_msgs/msg/Lane.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/behaviour_state_machine
)

### Generating Module File
_generate_module_lisp(behaviour_state_machine
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/behaviour_state_machine
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(behaviour_state_machine_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(behaviour_state_machine_generate_messages behaviour_state_machine_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros/fantasyplus/xtcar/src/behaviour_state_machine/srv/GoalPose.srv" NAME_WE)
add_dependencies(behaviour_state_machine_generate_messages_lisp _behaviour_state_machine_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(behaviour_state_machine_genlisp)
add_dependencies(behaviour_state_machine_genlisp behaviour_state_machine_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS behaviour_state_machine_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(behaviour_state_machine
  "/home/ros/fantasyplus/xtcar/src/behaviour_state_machine/srv/GoalPose.srv"
  "${MSG_I_FLAGS}"
  "/home/ros/fantasyplus/xtcar/src/mpc/mpc_msgs/msg/Waypoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/ros/fantasyplus/xtcar/src/mpc/mpc_msgs/msg/Lane.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/behaviour_state_machine
)

### Generating Module File
_generate_module_nodejs(behaviour_state_machine
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/behaviour_state_machine
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(behaviour_state_machine_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(behaviour_state_machine_generate_messages behaviour_state_machine_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros/fantasyplus/xtcar/src/behaviour_state_machine/srv/GoalPose.srv" NAME_WE)
add_dependencies(behaviour_state_machine_generate_messages_nodejs _behaviour_state_machine_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(behaviour_state_machine_gennodejs)
add_dependencies(behaviour_state_machine_gennodejs behaviour_state_machine_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS behaviour_state_machine_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(behaviour_state_machine
  "/home/ros/fantasyplus/xtcar/src/behaviour_state_machine/srv/GoalPose.srv"
  "${MSG_I_FLAGS}"
  "/home/ros/fantasyplus/xtcar/src/mpc/mpc_msgs/msg/Waypoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/TwistStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/ros/fantasyplus/xtcar/src/mpc/mpc_msgs/msg/Lane.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/behaviour_state_machine
)

### Generating Module File
_generate_module_py(behaviour_state_machine
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/behaviour_state_machine
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(behaviour_state_machine_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(behaviour_state_machine_generate_messages behaviour_state_machine_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ros/fantasyplus/xtcar/src/behaviour_state_machine/srv/GoalPose.srv" NAME_WE)
add_dependencies(behaviour_state_machine_generate_messages_py _behaviour_state_machine_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(behaviour_state_machine_genpy)
add_dependencies(behaviour_state_machine_genpy behaviour_state_machine_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS behaviour_state_machine_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/behaviour_state_machine)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/behaviour_state_machine
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(behaviour_state_machine_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(behaviour_state_machine_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET mpc_msgs_generate_messages_cpp)
  add_dependencies(behaviour_state_machine_generate_messages_cpp mpc_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/behaviour_state_machine)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/behaviour_state_machine
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(behaviour_state_machine_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(behaviour_state_machine_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET mpc_msgs_generate_messages_eus)
  add_dependencies(behaviour_state_machine_generate_messages_eus mpc_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/behaviour_state_machine)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/behaviour_state_machine
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(behaviour_state_machine_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(behaviour_state_machine_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET mpc_msgs_generate_messages_lisp)
  add_dependencies(behaviour_state_machine_generate_messages_lisp mpc_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/behaviour_state_machine)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/behaviour_state_machine
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(behaviour_state_machine_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(behaviour_state_machine_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET mpc_msgs_generate_messages_nodejs)
  add_dependencies(behaviour_state_machine_generate_messages_nodejs mpc_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/behaviour_state_machine)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/behaviour_state_machine\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/behaviour_state_machine
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(behaviour_state_machine_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(behaviour_state_machine_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET mpc_msgs_generate_messages_py)
  add_dependencies(behaviour_state_machine_generate_messages_py mpc_msgs_generate_messages_py)
endif()
