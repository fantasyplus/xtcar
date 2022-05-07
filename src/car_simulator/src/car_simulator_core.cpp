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

#include <string>
#include <algorithm>
#include "car_simulator/car_simulator_core.h"

WFSimulatorCore::WFSimulatorCore() : nh_(""), pnh_("~"), vehicle_sim_model_(nh_, pnh_)
{
  /* car_simulator parameters */
  double loop_rate;
  pnh_.param<double>("loop_rate", loop_rate, 50.0);
  pnh_.param<double>("lidar_height", lidar_height_, 0.6);
  pnh_.param<std::string>("simulation_frame_id", simulation_frame_id_, "base_link");
  pnh_.param<std::string>("map_frame_id", map_frame_id_, "map");
  pnh_.param<std::string>("lidar_frame_id", lidar_frame_id_, "rslidar");
  const double dt = 1.0 / loop_rate;

  /* set pub sub topic name */
  std::string sim_pose_name, sim_lidar_pose_name, sim_velocity_name, sim_vehicle_status_name;
  pnh_.param<std::string>("sim_pose_name", sim_pose_name, "gnss_pose");
  pnh_.param<std::string>("sim_velocity_name", sim_velocity_name, "current_velocity");
  pnh_.param<std::string>("sim_vehicle_status_name", sim_vehicle_status_name, "vehicle_status");

  pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(sim_pose_name, 1);
  pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped>(sim_velocity_name, 1);
  pub_vehicle_status_ = nh_.advertise<autoware_msgs::VehicleStatus>(sim_vehicle_status_name, 1);
  
  sub_vehicle_cmd_ = nh_.subscribe("vehicle_cmd", 1, &WFSimulatorCore::callbackVehicleCmd, this);
  timer_simulation_ = nh_.createTimer(ros::Duration(dt), &WFSimulatorCore::timerCallbackSimulation, this);


  // bool use_waypoints_for_z_position_source;
  // pnh_.param<bool>("use_waypoints_for_z_position_source", use_waypoints_for_z_position_source, false);
  // if (use_waypoints_for_z_position_source)
  // {
  //   sub_waypoints_ = nh_.subscribe("base_waypoints", 1, &WFSimulatorCore::callbackWaypoints, this);
  //   sub_closest_waypoint_ = nh_.subscribe("closest_waypoint", 1, &WFSimulatorCore::callbackClosestWaypoint, this);
  // }

  /* set initialize source */
  std::string initialize_source;
  pnh_.param<std::string>("initialize_source", initialize_source, "ORIGIN");
  ROS_INFO_STREAM("initialize_source : " << initialize_source);
  if (initialize_source == "RVIZ")
  {
    sub_initialpose_ = nh_.subscribe("initialpose", 1, &WFSimulatorCore::callbackInitialPoseWithCov, this);
  }
  else if (initialize_source == "GNSS")
  {
    sub_initialpose_ = nh_.subscribe("gnss_pose", 1, &WFSimulatorCore::callbackInitialPoseStamped, this);
  }
  else
  {
    ROS_WARN("initialize_source is undesired, setting error!!");
  }
  current_pose_.orientation.w = 1.0;

  closest_pos_z_ = 0.0;
}

// void WFSimulatorCore::callbackWaypoints(const autoware_msgs::LaneConstPtr& msg)
// {
//   current_waypoints_ptr_ = std::make_shared<autoware_msgs::Lane>(*msg);
// }

// void WFSimulatorCore::callbackClosestWaypoint(const std_msgs::Int32ConstPtr& msg)
// {
//   if (current_waypoints_ptr_ != nullptr)
//   {
//     if (-1 < msg->data && msg->data < static_cast<int>(current_waypoints_ptr_->waypoints.size()))
//     {
//       closest_pos_z_ = current_waypoints_ptr_->waypoints.at(msg->data).pose.pose.position.z;
//     }
//   }
// }

void WFSimulatorCore::callbackInitialPoseWithCov(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  geometry_msgs::Twist initial_twist;  // initialized with zero for all components
  setInitialStateWithPoseTransform(*msg, initial_twist);
}

void WFSimulatorCore::callbackInitialPoseStamped(const geometry_msgs::PoseStampedConstPtr& msg)
{
  geometry_msgs::Twist initial_twist;  // initialized with zero for all components
  setInitialStateWithPoseTransform(*msg, initial_twist);
}

void WFSimulatorCore::timerCallbackSimulation(const ros::TimerEvent& e)
{
  if (!vehicle_sim_model_.isInitialized())
  {
    ROS_INFO_DELAYED_THROTTLE(3.0, "[car_simulator] waiting initial position...");
    return;
  }

  if (prev_update_time_ptr_ == nullptr)
  {
    prev_update_time_ptr_ = std::make_shared<ros::Time>(ros::Time::now());
  }

  /* calculate delta time */
  const double dt = (ros::Time::now() - *prev_update_time_ptr_).toSec();
  *prev_update_time_ptr_ = ros::Time::now();

  /* update vehicle dynamics */
  vehicle_sim_model_.updateStatus(dt, closest_pos_z_);
  current_pose_ = vehicle_sim_model_.getCurrentPose();
  const geometry_msgs::Twist& current_twist = vehicle_sim_model_.getCurrentTwist();
  const double steering_angle = vehicle_sim_model_.getCurrentSteeringAngle();

  /* publish pose & twist */
  publishPoseTwist(current_pose_, current_twist);

  /* publish vehicle_statue for steering vehicle */
  autoware_msgs::VehicleStatus vs;
  vs.header.stamp = ros::Time::now();
  vs.header.frame_id = simulation_frame_id_;
  static const double MPS2KMPH = 3600.0 / 1000.0;
  vs.speed = current_twist.linear.x * MPS2KMPH;
  vs.angle = steering_angle;
  pub_vehicle_status_.publish(vs);
}

void WFSimulatorCore::callbackVehicleCmd(const autoware_msgs::VehicleCmdConstPtr& msg)
{
  vehicle_sim_model_.setVehicleCmd(msg);
}

void WFSimulatorCore::setInitialStateWithPoseTransform(const geometry_msgs::PoseStamped& pose_stamped,
                                                   const geometry_msgs::Twist& twist)
{
  tf::StampedTransform transform(getTransformFromTF(map_frame_id_, pose_stamped.header.frame_id));
  geometry_msgs::Pose pose;
  pose.position.x = pose_stamped.pose.position.x + transform.getOrigin().x();
  pose.position.y = pose_stamped.pose.position.y + transform.getOrigin().y();
  pose.position.z = pose_stamped.pose.position.z + transform.getOrigin().z();
  pose.orientation = pose_stamped.pose.orientation;
  vehicle_sim_model_.setInitialState(pose, twist);
}

void WFSimulatorCore::setInitialStateWithPoseTransform(const geometry_msgs::PoseWithCovarianceStamped& pose,
                                                   const geometry_msgs::Twist& twist)
{
  geometry_msgs::PoseStamped ps;
  ps.header = pose.header;
  ps.pose = pose.pose.pose;
  setInitialStateWithPoseTransform(ps, twist);
}

tf::StampedTransform WFSimulatorCore::getTransformFromTF(
  const std::string parent_frame, const std::string child_frame)
{
  tf::StampedTransform transform;
  while (1)
  {
    try
    {
      tf_listener_.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
      break;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }
  return transform;
}

void WFSimulatorCore::publishPoseTwist(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& twist)
{
  ros::Time current_time = ros::Time::now();

  // simulatied current_pose
  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = map_frame_id_;
  ps.header.stamp = current_time;
  ps.pose = pose;
  pub_pose_.publish(ps);

  geometry_msgs::TwistStamped ts;
  ts.header.frame_id = simulation_frame_id_;
  ts.header.stamp = current_time;
  ts.twist = twist;
  pub_twist_.publish(ts);
}