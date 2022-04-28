#include "hybrid_astar_node.h"

geometry_msgs::Pose transformPose(const geometry_msgs::Pose &pose, const geometry_msgs::TransformStamped &transform)
{
  geometry_msgs::PoseStamped transformed_pose;
  geometry_msgs::PoseStamped orig_pose;
  orig_pose.pose = pose;
  tf2::doTransform(orig_pose, transformed_pose, transform);

  return transformed_pose.pose;
}

geometry_msgs::TransformStamped HybridAstarNode::getTransform(const string &target, const string &source)
{
  geometry_msgs::TransformStamped tf;
  try
  {
    // tf_buffer_->setUsingDedicatedThread(true);
    tf = tf_buffer_->lookupTransform(target, source, ros::Time(0), ros::Duration(1));
  }
  catch (const tf2::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  return tf;
}

HybridAstarNode::HybridAstarNode() : nh_(), private_nh_("~")
{
  //节点参数
  private_nh_.param<std::string>("costmap_topic", _costmap_topic, "global_cost_map");
  private_nh_.param<std::string>("pose_topic", _pose_topic, "current_pose");
  private_nh_.param<double>("waypoints_velocity", waypoints_velocity_, 5.0);
  private_nh_.param<double>("update_rate", update_rate_, 10.0);
  private_nh_.param<bool>("is_visual", is_visual, false);
  private_nh_.param<bool>("use_rviz_start", use_rviz_start, false);

  //混合A*参数
  private_nh_.param<double>("time_limit", _hybrid_astar_param.time_limit, 10000.0);

  private_nh_.param<double>("vehicle_length", _hybrid_astar_param.vehiche_shape.length, 4.788);
  private_nh_.param<double>("vehicle_width", _hybrid_astar_param.vehiche_shape.width, 2.198);
  private_nh_.param<double>("vehicle_cg2back", _hybrid_astar_param.vehiche_shape.cg2back, 1.367);

  private_nh_.param<double>("max_turning_radius", _hybrid_astar_param.max_turning_radius, 20.0);
  private_nh_.param<double>("min_turning_radius", _hybrid_astar_param.min_turning_radius, 4.36);
  private_nh_.param<int>("turning_radius_size", _hybrid_astar_param.turning_radius_size, 11);
  private_nh_.param<int>("theta_size", _hybrid_astar_param.theta_size, 48);

  private_nh_.param<double>("reverse_weight", _hybrid_astar_param.reverse_weight, 4.0);
  private_nh_.param<double>("turning_weight", _hybrid_astar_param.turning_weight, 2.2);
  private_nh_.param<double>("goal_lateral_tolerance", _hybrid_astar_param.goal_lateral_tolerance, 1.0);
  private_nh_.param<double>("goal_longitudinal_tolerance", _hybrid_astar_param.goal_longitudinal_tolerance, 1.0);
  private_nh_.param<double>("goal_angular_tolerance", _hybrid_astar_param.goal_angular_tolerance, 0.15708);
  private_nh_.param<int>("obstacle_threshold", _hybrid_astar_param.obstacle_threshold, 100);

  private_nh_.param<bool>("use_back", _hybrid_astar_param.use_back, true);
  private_nh_.param<bool>("use_reeds_shepp", _hybrid_astar_param.use_reeds_shepp, true);
  private_nh_.param<bool>("use_smoother", _hybrid_astar_param.use_smoother, false);

  private_nh_.param<float>("alpha", _hybrid_astar_param.alpha, 0.1);
  private_nh_.param<float>("obstacle_weight", _hybrid_astar_param.obstacle_weight, 0.1);
  private_nh_.param<float>("curvature_weight", _hybrid_astar_param.curvature_weight, 0.0);
  private_nh_.param<float>("smoothness_weight", _hybrid_astar_param.smoothness_weight, 0.1);
  private_nh_.param<float>("obstacle_distance_max", _hybrid_astar_param.obstacle_distance_max, 1.0);

  private_nh_.param<float>("analytic_expansion_ratio", _hybrid_astar_param.analytic_expansion_ratio, 35);
  private_nh_.param<float>("analytic_expansion_max_length", _hybrid_astar_param.analytic_expansion_max_length, 300);

  _hybrid_astar.initParam(_hybrid_astar_param);

  costmap_sub_ = nh_.subscribe(_costmap_topic, 1, &HybridAstarNode::costmapCallback, this);
  current_pose_sub_ = nh_.subscribe(_pose_topic, 1, &HybridAstarNode::currentPoseCallback, this);
  if (use_rviz_start)
    rviz_start_sub_ = nh_.subscribe("initialpose", 1, &HybridAstarNode::currentRvizPoseCallback, this);
  goal_pose_sub_ = nh_.subscribe("move_base_simple/goal", 1, &HybridAstarNode::goalPoseCallback, this);

  _pub_initial_path = nh_.advertise<nav_msgs::Path>("initial_path", 1, true);
  _pub_smoothed_path = nh_.advertise<nav_msgs::Path>("smoothed_path", 1, true);
  _pub_path_vehicles = nh_.advertise<visualization_msgs::MarkerArray>("path_vehicle", 1, true);

  costmap_initialized_ = false;
  current_pose_initialized_ = false;
  goal_pose_initialized_ = false;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void HybridAstarNode::costmapCallback(const nav_msgs::OccupancyGrid &msg)
{
  costmap_ = msg;

  costmap_initialized_ = true;

  _hybrid_astar.SetOccupancyGrid(costmap_);
}

void HybridAstarNode::currentRvizPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  if (!costmap_initialized_)
  {
    return;
  }
  geometry_msgs::PoseStamped start_pose;
  start_pose.pose = msg->pose.pose;
  start_pose.header = msg->header;
  current_pose_global_ = start_pose;
  current_pose_initialized_ = true;
}

void HybridAstarNode::currentPoseCallback(const geometry_msgs::PoseStamped &msg)
{
  if (!costmap_initialized_)
  {
    return;
  }
  current_pose_global_ = msg;
  current_pose_initialized_ = true;
}

void HybridAstarNode::goalPoseCallback(const geometry_msgs::PoseStamped &msg)
{
  if (!costmap_initialized_)
  {
    return;
  }
  goal_pose_global_ = msg;
  goal_pose_initialized_ = true;
}

inline bool isSuccess(const SearchStatus &status)
{
  return status == SearchStatus::SUCCESS;
}

void HybridAstarNode::run()
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

    const auto start_pose_in_costmap_frame = transformPose(
        current_pose_global_.pose,
        getTransform(costmap_.header.frame_id, current_pose_global_.header.frame_id));

    const auto goal_pose_in_costmap_frame = transformPose(
        goal_pose_global_.pose,
        getTransform(costmap_.header.frame_id, goal_pose_global_.header.frame_id));

    TimerClock t0;
    SearchStatus result = _hybrid_astar.makePlan(start_pose_in_costmap_frame, goal_pose_in_costmap_frame);
    ROS_INFO("Hybrid Astar planning: %f [ms]", t0.getTimerMilliSec());

    if (isSuccess(result))
    {
      ROS_INFO("Plan found.");
      TrajectoryWaypoints waypoints = _hybrid_astar.getTrajectory();

      _hybrid_astar.visualAnalyticPath();
      if (is_visual)
      {
        // _hybrid_astar.visualCollisionClear();
        visualPathVehicle(waypoints);
        _hybrid_astar.visualOpenNode();
      }

      //初始路径
      nav_msgs::Path initial_traj = transferTrajectory(current_pose_global_.pose, waypoints);
      _pub_initial_path.publish(initial_traj);

      //平滑后的路径
      if (_hybrid_astar_param.use_smoother)
      {
        TimerClock t1;
        _hybrid_astar.smoothPath(waypoints);
        ROS_INFO("smooth path cost: %f [ms]", t1.getTimerMilliSec());

        nav_msgs::Path smooth_traj = transferTrajectory(current_pose_global_.pose, waypoints);
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

nav_msgs::Path HybridAstarNode::transferTrajectory(const geometry_msgs::Pose &current_pose,
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

void HybridAstarNode::visualPathVehicle(const TrajectoryWaypoints &visual_waypoints)
{
  int clear_index = 0;
  static int id = 0;

  for (int i = 0; i < visual_waypoints.trajectory.size(); i++)
  {
    visualization_msgs::Marker vehicle_marker;

    if (clear_index == 0)
    {
      vehicle_marker.action = 3;
      clear_index = 1;
    }

    vehicle_marker.header.frame_id = costmap_.header.frame_id;
    vehicle_marker.header.stamp = ros::Time::now();
    vehicle_marker.id = id++;
    vehicle_marker.type = visualization_msgs::Marker::CUBE;

    vehicle_marker.scale.x = _hybrid_astar_param.vehiche_shape.length;
    vehicle_marker.scale.y = _hybrid_astar_param.vehiche_shape.width;
    vehicle_marker.scale.z = 1.0;

    vehicle_marker.color.a = 0.2;

    if (i != visual_waypoints.trajectory.size() - 1 && visual_waypoints.trajectory[i].is_back != visual_waypoints.trajectory[i + 1].is_back)
    {
      vehicle_marker.color.r = black.red;
      vehicle_marker.color.r = black.green;
      vehicle_marker.color.r = black.blue;
    }
    else if (visual_waypoints.trajectory[i].is_back == false)
    {
      vehicle_marker.color.r = green.red;
      vehicle_marker.color.g = green.green;
      vehicle_marker.color.b = green.blue;
    }
    else if (visual_waypoints.trajectory[i].is_back == true)
    {
      vehicle_marker.color.r = pink.red;
      vehicle_marker.color.g = pink.green;
      vehicle_marker.color.b = pink.blue;
    }

    vehicle_marker.pose.position.x = visual_waypoints.trajectory[i].pose.pose.position.x;
    vehicle_marker.pose.position.y = visual_waypoints.trajectory[i].pose.pose.position.y;
    vehicle_marker.pose.orientation = visual_waypoints.trajectory[i].pose.pose.orientation;

    _path_vehicles.markers.push_back(vehicle_marker);
  }

  _pub_path_vehicles.publish(_path_vehicles);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "astar_navi");

  HybridAstarNode node;
  node.run();

  return 0;
}