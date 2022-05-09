cmake_minimum_required(VERSION 3.0.2)
project(mpc_follower)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mpc_msgs
  qpoases_vendor
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
)

set(
  MPC_FOLLOWER_SRC
    src/mpc_utils.cpp
    src/mpc_trajectory.cpp
    src/lowpass_filter.cpp
    src/mpc_follower_core.cpp
    src/vehicle_model/vehicle_model_interface.cpp
    src/vehicle_model/vehicle_model_bicycle_kinematics.cpp
    src/vehicle_model/vehicle_model_bicycle_dynamics.cpp
    src/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.cpp
    src/qp_solver/qp_solver_unconstr.cpp
    src/qp_solver/qp_solver_unconstr_fast.cpp
    src/qp_solver/qp_solver_qpoases.cpp
    src/amathutils_lib/Amathutils.cpp
)

# set(QPOASES_INLUDE_DIRS /usr/local/include/qpOASES)

include_directories(
  include
  # ${QPOASES_INLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${qpoases_vendor_INCLUDE_DIRS}/qpoases_vendor
)



add_executable(mpc_save_traj src/mpc_save_traj.cpp)
target_link_libraries(mpc_save_traj ${catkin_LIBRARIES})

add_executable(mpc_read_traj src/mpc_read_traj.cpp)
target_link_libraries(mpc_read_traj ${catkin_LIBRARIES})

add_executable(try src/try.cpp ${MPC_FOLLOWER_SRC})
target_link_libraries(try ${catkin_LIBRARIES})

add_executable(mpc_follower src/mpc_follower_node.cpp ${MPC_FOLLOWER_SRC})
target_link_libraries(mpc_follower ${catkin_LIBRARIES})
