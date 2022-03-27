/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "hp_hybrid_astr_node/astar_navi.h"

inline geometry_msgs::Pose transformPose(const geometry_msgs::Pose &pose, const tf::Transform &tf)
{
  // Convert ROS pose to TF pose
  tf::Pose tf_pose;
  tf::poseMsgToTF(pose, tf_pose);

  // Transform pose
  tf_pose = tf * tf_pose;

  // Convert TF pose to ROS pose
  geometry_msgs::Pose ros_pose;
  tf::poseTFToMsg(tf_pose, ros_pose);

  return ros_pose;
}

AstarNavi::AstarNavi() : nh_(), private_nh_("~")
{
  // node param
  private_nh_.param<std::string>("costmap_topic", _costmap_topic, "global_cost_map");
  private_nh_.param<double>("waypoints_velocity", waypoints_velocity_, 5.0);
  private_nh_.param<double>("update_rate", update_rate_, 10.0);
  private_nh_.param<bool>("is_visual", is_visual, false);

  /*-----hybrid astar param-----*/
  private_nh_.param<double>("time_limit", _hybrid_astar_param.time_limit, 10000.0);

  private_nh_.param<double>("vehicle_length", _hybrid_astar_param.vehiche_shape.length, 4.788);
  private_nh_.param<double>("vehicle_width", _hybrid_astar_param.vehiche_shape.width, 2.198);
  private_nh_.param<double>("vehicle_cg2back", _hybrid_astar_param.vehiche_shape.cg2back, 1.367);

  private_nh_.param<double>("max_turning_radius", _hybrid_astar_param.max_turning_radius, 20.0);
  private_nh_.param<double>("min_turning_radius", _hybrid_astar_param.min_turning_radius, 4.36); // wheel_base/max_turning_ras
  private_nh_.param<int>("turning_radius_size", _hybrid_astar_param.turning_radius_size, 11);
  private_nh_.param<int>("theta_size", _hybrid_astar_param.theta_size, 72);

  private_nh_.param<double>("reverse_weight", _hybrid_astar_param.reverse_weight, 4.0);
  private_nh_.param<double>("turning_weight", _hybrid_astar_param.turning_weight, 2.2);
  private_nh_.param<double>("goal_lateral_tolerance", _hybrid_astar_param.goal_lateral_tolerance, 1.0);
  private_nh_.param<double>("goal_longitudinal_tolerance", _hybrid_astar_param.goal_longitudinal_tolerance, 1.0);
  private_nh_.param<double>("goal_angular_tolerance", _hybrid_astar_param.goal_angular_tolerance, 0.15708);
  private_nh_.param<int>("obstacle_threshold", _hybrid_astar_param.obstacle_threshold, 100);

  private_nh_.param<bool>("use_back", _hybrid_astar_param.use_back, true);
  private_nh_.param<bool>("use_reeds_shepp", _hybrid_astar_param.use_reeds_shepp, true);
  private_nh_.param<bool>("use_smoother", _hybrid_astar_param.use_smoother, true);

  private_nh_.param<float>("alpha", _hybrid_astar_param.alpha, 0.1);
  private_nh_.param<float>("obstacle_weight", _hybrid_astar_param.obstacle_weight, 0.1);
  private_nh_.param<float>("curvature_weight", _hybrid_astar_param.curvature_weight, 0.0);
  private_nh_.param<float>("smoothness_weight", _hybrid_astar_param.smoothness_weight, 0.1);
  private_nh_.param<float>("obstacle_distance_max", _hybrid_astar_param.obstacle_distance_max, 1.0);

  private_nh_.param<float>("analytic_expansion_ratio", _hybrid_astar_param.analytic_expansion_ratio, 35);
  private_nh_.param<float>("analytic_expansion_max_length", _hybrid_astar_param.analytic_expansion_max_length, 300);

  _hybrid_astar.initParam(_hybrid_astar_param);

  lane_pub_ = nh_.advertise<autoware_msgs::LaneArray>("lane_waypoints_array", 1, true);
  costmap_sub_ = nh_.subscribe(_costmap_topic, 1, &AstarNavi::costmapCallback, this);
  current_pose_sub_ = nh_.subscribe("current_pose", 1, &AstarNavi::currentPoseCallback, this);
  goal_pose_sub_ = nh_.subscribe("move_base_simple/goal", 1, &AstarNavi::goalPoseCallback, this);

  pub_PathsRviz = nh_.advertise<visualization_msgs::MarkerArray>("global_waypoints_rviz", 1, true);
  _pub_initial_path = nh_.advertise<nav_msgs::Path>("initial_path", 1, true);
  _pub_smoothed_path = nh_.advertise<nav_msgs::Path>("smoothed_path", 1, true);

  costmap_initialized_ = false;
  current_pose_initialized_ = false;
  goal_pose_initialized_ = false;
}

AstarNavi::~AstarNavi()
{
}

void AstarNavi::costmapCallback(const nav_msgs::OccupancyGrid &msg)
{
  costmap_ = msg;
  tf::poseMsgToTF(costmap_.info.origin, local2costmap_);

  costmap_initialized_ = true;

  _hybrid_astar.SetOccupancyGrid(costmap_);
}

void AstarNavi::currentPoseCallback(const geometry_msgs::PoseStamped &msg)
{
  if (!costmap_initialized_)
  {
    return;
  }

  current_pose_global_ = msg;
  current_pose_local_.pose = transformPose(current_pose_global_.pose, getTransform(costmap_.header.frame_id, current_pose_global_.header.frame_id));
  current_pose_local_.header.frame_id = costmap_.header.frame_id;
  current_pose_local_.header.stamp = current_pose_global_.header.stamp;

  current_pose_initialized_ = true;
}

void AstarNavi::goalPoseCallback(const geometry_msgs::PoseStamped &msg)
{
  if (!costmap_initialized_)
  {
    return;
  }

  goal_pose_global_ = msg;
  goal_pose_local_.pose = transformPose(goal_pose_global_.pose, getTransform(costmap_.header.frame_id, goal_pose_global_.header.frame_id));
  goal_pose_local_.header.frame_id = costmap_.header.frame_id;
  goal_pose_local_.header.stamp = goal_pose_global_.header.stamp;

  goal_pose_initialized_ = true;
}

tf::Transform AstarNavi::getTransform(const std::string &from, const std::string &to)
{
  tf::StampedTransform stf;
  try
  { // from是target，to是source，他这个写的有误导性
    tf_listener_.lookupTransform(from, to, ros::Time(0), stf);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  return stf;
}

inline bool isSuccess(const SearchStatus &status)
{
  return status == SearchStatus::SUCCESS;
}

void AstarNavi::run()
{
  ros::Rate rate(update_rate_);

  while (ros::ok())
  {
    ros::spinOnce();

    if (!costmap_initialized_ || !current_pose_initialized_ || !goal_pose_initialized_)
    {
      rate.sleep();
      continue;
    }

    goalPoseCallback(goal_pose_global_);

    TimerClock t0;
    SearchStatus result = _hybrid_astar.makePlan(current_pose_local_.pose, goal_pose_local_.pose);
    ROS_INFO("Hybrid Astar planning: %f [ms]", t0.getTimerMilliSec());

    if (isSuccess(result))
    {
      ROS_INFO("Plan found.");
      TrajectoryWaypoints waypoints = _hybrid_astar.getTrajectory();

      if (is_visual)
      {
        // _hybrid_astar.visualCollisionClear();
        _hybrid_astar.visual();
      }

      //初始路径
      nav_msgs::Path initial_traj = transferTrajectory(current_pose_local_.pose, waypoints);
      _pub_initial_path.publish(initial_traj);

      //平滑后的路径
      if (_hybrid_astar_param.use_smoother)
      {
        TimerClock t1;
        _hybrid_astar.smoothPath(waypoints);
        ROS_INFO("smooth path cost: %f [ms]", t1.getTimerMilliSec());

        nav_msgs::Path smooth_traj = transferTrajectory(current_pose_local_.pose, waypoints);
        _pub_smoothed_path.publish(smooth_traj);
      }
    }
    else
    {
      switch (result)
      {
      case SearchStatus::FAILURE_COLLISION_AT_START:
        ROS_INFO("Cannot find plan because collision was detected in start position.");
        break;
      case SearchStatus::FAILURE_COLLISION_AT_GOAL:
        ROS_INFO("Cannot find plan because collision was detected in goal position.");
        break;
      case SearchStatus::FAILURE_TIMEOUT_EXCEEDED:
        ROS_INFO("Cannot find plan because timeout exceeded.");
        break;
      case SearchStatus::FAILURE_NO_PATH_FOUND:
        ROS_INFO("Cannot find plan.");
        break;
      default:
        ROS_INFO("SearchStatus not handled.");
        break;
      }
    }

    TimerClock t2;
    _hybrid_astar.reset();
    ROS_INFO("reset cost: %f [ms]", t2.getTimerMilliSec());

    rate.sleep();
  }
}

nav_msgs::Path AstarNavi::transferTrajectory(const geometry_msgs::Pose &current_pose,
                                             const TrajectoryWaypoints &trajectory_waypoints)
{
  nav_msgs::Path ros_traj;
  ros_traj.header = trajectory_waypoints.header;

  for (auto single_wp : trajectory_waypoints.trajectory)
  {
    geometry_msgs::PoseStamped temp_pose;

    temp_pose.header = trajectory_waypoints.header;

    temp_pose.pose.position.x = single_wp.pose.pose.position.x;
    temp_pose.pose.position.y = single_wp.pose.pose.position.y;
    temp_pose.pose.position.z = single_wp.pose.pose.position.z;

    temp_pose.pose.orientation = single_wp.pose.pose.orientation;

    ros_traj.poses.push_back(temp_pose);
  }

  return ros_traj;
}

void AstarNavi::publishWaypoints(const nav_msgs::Path &path, const double &velocity)
{
  autoware_msgs::Lane lane;
  lane.header.frame_id = "map";
  lane.header.stamp = path.header.stamp;
  lane.increment = 0;

  for (const auto &pose : path.poses)
  {
    autoware_msgs::Waypoint wp;
    wp.pose.header = lane.header;
    wp.pose.pose = transformPose(pose.pose, getTransform(lane.header.frame_id, pose.header.frame_id));
    wp.pose.pose.position.z = current_pose_global_.pose.position.z; // height = const
    wp.twist.twist.linear.x = velocity / 3.6;                       // velocity = const
    lane.waypoints.push_back(wp);
  }

  autoware_msgs::LaneArray lane_array;
  lane_array.lanes.push_back(lane);
  lane_pub_.publish(lane_array);

  visualization_msgs::MarkerArray pathsToVisualize;

  std_msgs::ColorRGBA total_color;
  total_color.r = 0;
  total_color.g = 0.7;
  total_color.b = 1.0;
  total_color.a = 0.9;
  PlannerHNS::ROSHelpers::createGlobalLaneArrayMarker(total_color, lane_array, pathsToVisualize);
  PlannerHNS::ROSHelpers::createGlobalLaneArrayOrientationMarker(lane_array, pathsToVisualize);
  PlannerHNS::ROSHelpers::createGlobalLaneArrayVelocityMarker(lane_array, pathsToVisualize);
  pub_PathsRviz.publish(pathsToVisualize);
}
