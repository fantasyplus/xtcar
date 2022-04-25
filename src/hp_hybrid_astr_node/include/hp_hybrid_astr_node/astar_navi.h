/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#ifndef ASTAR_NAVI_H
#define ASTAR_NAVI_H

#include <iostream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
// #include <autoware_msgs/LaneArray.h>

// #include "astar_search/astar_search.h"
#include "high_performence_hybrid_astar.hpp"

// #include "op_ros_helpers/op_ROSHelpers.h"
class AstarNavi
{
public:
  AstarNavi();
  ~AstarNavi();
  void run();

private:
  // ros
  ros::NodeHandle nh_, private_nh_;

  ros::Publisher _pub_initial_path;
  ros::Publisher _pub_smoothed_path;

  ros::Subscriber costmap_sub_;
  ros::Subscriber current_pose_sub_;
  ros::Subscriber goal_pose_sub_;
  ros::Subscriber rviz_start_sub_;

  double waypoints_velocity_; // constant velocity on planned waypoints [km/h]
  double update_rate_;        // replanning and publishing rate [Hz]
  bool is_visual;
  std::string _costmap_topic;

  // classes
  // AstarSearch astar_;
  HybridAstar _hybrid_astar;
  PlannerCommonParam _hybrid_astar_param;

  // variables
  nav_msgs::OccupancyGrid costmap_;
  geometry_msgs::PoseStamped current_pose_global_;
  geometry_msgs::PoseStamped goal_pose_global_;
  tf::Transform local2costmap_; // local frame (e.g. velodyne) -> costmap origin

  bool costmap_initialized_;
  bool current_pose_initialized_;
  bool goal_pose_initialized_;

  void costmapCallback(const nav_msgs::OccupancyGrid &msg);
  void currentPoseCallback(const geometry_msgs::PoseStamped &msg);
  void currentRvizPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void goalPoseCallback(const geometry_msgs::PoseStamped &msg);

  nav_msgs::Path transferTrajectory(const geometry_msgs::Pose &current_pose,
                                    const TrajectoryWaypoints &trajectory_waypoints);

private:
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

  void visualPathVehicle(const TrajectoryWaypoints &visual_waypoints);
};

#endif
