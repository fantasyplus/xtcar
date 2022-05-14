#ifndef HYBRID_ASTAR_NODE_H
#define HYBRID_ASTAR_NODE_H

// std C++
#include <iostream>
#include <vector>
#include <string>

// ros
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

//自定义
#include "high_performence_hybrid_astar.hpp"
#include "behaviour_state_machine/GoalPose.h"
#include "mpc_msgs/Lane.h"
#include "mpc_msgs/Waypoint.h"
#include "mpc_msgs/ControlCommand.h"
#include "mpc_msgs/VehicleStatus.h"

class HybridAstarNode
{
public:
  HybridAstarNode();
  void run();

private:
  /*---------------------ros相关---------------------*/
  ros::NodeHandle _nh, _private_nh;

  ros::Publisher _pub_initial_path;
  ros::Publisher _pub_smoothed_path;

  ros::Subscriber _costmap_sub;
  ros::Subscriber _current_pose_sub;
  ros::Subscriber _goal_pose_sub_;
  ros::Subscriber _rviz_start_sub;

  ros::ServiceServer _goal_pose_server;

  void costmapCallback(const nav_msgs::OccupancyGrid &msg);
  void currentPoseCallback(const geometry_msgs::PoseStamped &msg);
  void currentRvizPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void goalPoseCallback(const geometry_msgs::PoseStamped &msg);

  bool srvHandleGoalPose(behaviour_state_machine::GoalPose::Request &req, behaviour_state_machine::GoalPose::Response &res);

private:
  /*---------------------该节点的参数(不是Hybrid A*的)---------------------*/
  double _waypoints_velocity;
  double _update_rate;
  bool is_visual;
  bool use_rviz_start;
  std::string _costmap_topic;
  std::string _pose_topic;

  HybridAstar _hybrid_astar;
  PlannerCommonParam _hybrid_astar_param;

  nav_msgs::OccupancyGrid _costmap;
  geometry_msgs::PoseStamped _current_pose_global;
  geometry_msgs::PoseStamped _goal_pose_global;

  bool _costmap_initialized, _current_pose_initialized, _goal_pose_initialized;

  inline bool isSuccess(const SearchStatus &status);

private:
  /*---------------------tf变换相关---------------------*/
  std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

  geometry_msgs::TransformStamped getTransform(const string &target, const string &source);

  nav_msgs::Path transferTrajectory(const geometry_msgs::Pose &current_pose,
                                    const TrajectoryWaypoints &trajectory_waypoints);

private:
  /*---------------------用于可视化最终轨迹---------------------*/
  ros::Publisher _pub_path_vehicles;
  visualization_msgs::MarkerArray _path_vehicles; //车子数据结构，用于可视化

  struct color
  {
    float red;
    float green;
    float blue;
  };

  static constexpr color teal = {102.f / 255.f, 217.f / 255.f, 239.f / 255.f};

  static constexpr color green = {166.f / 255.f, 226.f / 255.f, 46.f / 255.f};

  static constexpr color orange = {253.f / 255.f, 151.f / 255.f, 31.f / 255.f};

  static constexpr color pink = {249.f / 255.f, 38.f / 255.f, 114.f / 255.f};

  static constexpr color purple = {174.f / 255.f, 129.f / 255.f, 255.f / 255.f};

  static constexpr color black = {0.f / 255.f, 0.f / 255.f, 0.f / 255.f};

  static constexpr color blue = {51.f / 255.f, 204.f / 255.f, 204.f / 255.f};

  void visualPathVehicle(const TrajectoryWaypoints &visual_waypoints);

private:
  /*---------------------suv car---------------------*/
  mpc_msgs::Lane convertToMpcMsgsLane(const TrajectoryWaypoints &traj);
};

#endif
