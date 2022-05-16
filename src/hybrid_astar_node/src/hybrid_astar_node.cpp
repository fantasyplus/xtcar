#include "hybrid_astar_node.h"

geometry_msgs::Pose transformPose(const geometry_msgs::Pose &pose,
                                  const geometry_msgs::TransformStamped &transform)
{
  geometry_msgs::PoseStamped transformed_pose;
  geometry_msgs::PoseStamped orig_pose;
  orig_pose.pose = pose;
  tf2::doTransform(orig_pose, transformed_pose, transform);

  return transformed_pose.pose;
}

geometry_msgs::TransformStamped HybridAstarNode::getTransform(const string &target,
                                                              const string &source)
{
  geometry_msgs::TransformStamped tf;
  try
  {
    // _tf_buffer->setUsingDedicatedThread(true);
    tf = _tf_buffer->lookupTransform(target, source, ros::Time(0), ros::Duration(1));
  }
  catch (const tf2::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  return tf;
}

HybridAstarNode::HybridAstarNode() : _nh(), _private_nh("~")
{
  //节点参数
  _private_nh.param<std::string>("costmap_topic", _costmap_topic, "global_cost_map");
  _private_nh.param<std::string>("pose_topic", _pose_topic, "gnss_pose");
  _private_nh.param<double>("waypoints_velocity", _waypoints_velocity, 2.0);
  _private_nh.param<double>("update_rate", _update_rate, 10.0);
  _private_nh.param<bool>("is_visual", is_visual, true);
  _private_nh.param<bool>("use_rviz_start", use_rviz_start, false);

  //混合A*参数
  _private_nh.param<double>("time_limit", _hybrid_astar_param.time_limit, 10000.0);

  _private_nh.param<double>("vehicle_length", _hybrid_astar_param.vehiche_shape.length, 4.788);
  _private_nh.param<double>("vehicle_width", _hybrid_astar_param.vehiche_shape.width, 2.198);
  _private_nh.param<double>("vehicle_cg2back", _hybrid_astar_param.vehiche_shape.cg2back, 1.367);

  _private_nh.param<double>("max_turning_radius", _hybrid_astar_param.max_turning_radius, 20.0);
  _private_nh.param<double>("min_turning_radius", _hybrid_astar_param.min_turning_radius, 4.36);
  _private_nh.param<int>("turning_radius_size", _hybrid_astar_param.turning_radius_size, 1);
  _private_nh.param<int>("theta_size", _hybrid_astar_param.theta_size, 48);

  _private_nh.param<double>("reverse_weight", _hybrid_astar_param.reverse_weight, 4.0);
  _private_nh.param<double>("turning_weight", _hybrid_astar_param.turning_weight, 2.2);
  _private_nh.param<double>("goal_lateral_tolerance", _hybrid_astar_param.goal_lateral_tolerance, 0.5);
  _private_nh.param<double>("goal_longitudinal_tolerance", _hybrid_astar_param.goal_longitudinal_tolerance, 0.5);
  _private_nh.param<double>("goal_angular_tolerance", _hybrid_astar_param.goal_angular_tolerance, 0.15708);
  _private_nh.param<int>("obstacle_threshold", _hybrid_astar_param.obstacle_threshold, 100);

  _private_nh.param<bool>("use_back", _hybrid_astar_param.use_back, true);
  _private_nh.param<bool>("use_reeds_shepp", _hybrid_astar_param.use_reeds_shepp, true);
  _private_nh.param<bool>("use_obstacle_heuristic", _hybrid_astar_param.use_obstacle_heuristic, false);
  _private_nh.param<bool>("use_analytic_expansion", _hybrid_astar_param.use_analytic_expansion, true);
  _private_nh.param<bool>("use_theta_cost", _hybrid_astar_param.use_theta_cost, false);
  _private_nh.param<double>("obstacle_theta_ratio", _hybrid_astar_param.obstacle_theta_ratio, 1.0);

  _private_nh.param<bool>("use_smoother", _hybrid_astar_param.use_smoother, false);

  _private_nh.param<float>("alpha", _hybrid_astar_param.alpha, 0.1);
  _private_nh.param<float>("obstacle_weight", _hybrid_astar_param.obstacle_weight, 0.1);
  _private_nh.param<float>("curvature_weight", _hybrid_astar_param.curvature_weight, 0.0);
  _private_nh.param<float>("smoothness_weight", _hybrid_astar_param.smoothness_weight, 0.1);
  _private_nh.param<float>("obstacle_distance_max", _hybrid_astar_param.obstacle_distance_max, 1.0);

  _private_nh.param<float>("analytic_expansion_ratio", _hybrid_astar_param.analytic_expansion_ratio, 35);
  _private_nh.param<float>("analytic_expansion_max_length", _hybrid_astar_param.analytic_expansion_max_length, 30);

  /*---------------------subscribe---------------------*/
  _costmap_sub = _nh.subscribe(_costmap_topic, 1, &HybridAstarNode::costmapCallback, this);
  //如果用rviz定起点的话，就是静态地图，不是的话就是动态地图实时定位
  if (use_rviz_start)
    _rviz_start_sub = _nh.subscribe("initialpose", 1, &HybridAstarNode::currentRvizPoseCallback, this);
  else
    _current_pose_sub = _nh.subscribe(_pose_topic, 1, &HybridAstarNode::currentPoseCallback, this);

  /*---------------------advertise---------------------*/
  _pub_initial_path = _nh.advertise<nav_msgs::Path>("initial_path", 1, true);
  _pub_smoothed_path = _nh.advertise<nav_msgs::Path>("smoothed_path", 1, true);
  _pub_path_vehicles = _nh.advertise<visualization_msgs::MarkerArray>("path_vehicle", 1, true);

  /*---------------------service_server---------------------*/
  _goal_pose_server = _nh.advertiseService("goal_pose_srv", &HybridAstarNode::srvHandleGoalPose, this);

  _costmap_initialized = false;
  _current_pose_initialized = false;
  _goal_pose_initialized = false;

  _tf_buffer = std::make_shared<tf2_ros::Buffer>();
  _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

  _hybrid_astar.initParam(_hybrid_astar_param);
}

void HybridAstarNode::costmapCallback(const nav_msgs::OccupancyGrid &msg)
{
  _costmap = msg;

  _costmap_initialized = true;

  _hybrid_astar.SetOccupancyGrid(_costmap);
}

void HybridAstarNode::currentRvizPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  if (!_costmap_initialized)
  {
    return;
  }

  _current_pose_global.pose = msg->pose.pose;
  _current_pose_global.header = msg->header;
  _current_pose_initialized = true;
}

void HybridAstarNode::currentPoseCallback(const geometry_msgs::PoseStamped &msg)
{
  if (!_costmap_initialized)
  {
    return;
  }
  _current_pose_global = msg;
  _current_pose_initialized = true;
}

bool HybridAstarNode::srvHandleGoalPose(behaviour_state_machine::GoalPose::Request &req,
                                        behaviour_state_machine::GoalPose::Response &res)
{
  _goal_pose_global.pose = req.pose;
  _goal_pose_global.header = req.header;

  const auto start_pose_in_costmap_frame = transformPose(_current_pose_global.pose,
                                                         getTransform(_costmap.header.frame_id, _current_pose_global.header.frame_id));
  // ROS_INFO("start_pose_in_costmap_frame:%f,%f", start_pose_in_costmap_frame.position.x, start_pose_in_costmap_frame.position.y);

  const auto goal_pose_in_costmap_frame = transformPose(_goal_pose_global.pose,
                                                        getTransform(_costmap.header.frame_id, _goal_pose_global.header.frame_id));
  // ROS_INFO("goal_pose_in_costmap_frame:%f,%f", goal_pose_in_costmap_frame.position.x, goal_pose_in_costmap_frame.position.y);

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
    nav_msgs::Path initial_traj = transferTrajectory(_current_pose_global.pose, waypoints);
    _pub_initial_path.publish(initial_traj);

    //平滑后的路径
    if (_hybrid_astar_param.use_smoother)
    {
      TimerClock t1;
      _hybrid_astar.smoothPath(waypoints);
      ROS_INFO("smooth path cost: %f [ms]", t1.getTimerMilliSec());

      nav_msgs::Path smooth_traj = transferTrajectory(_current_pose_global.pose, waypoints);
      _pub_smoothed_path.publish(smooth_traj);
    }

    //计算重置耗时
    TimerClock t2;
    _hybrid_astar.reset();
    ROS_INFO("reset cost: %f [ms]", t2.getTimerMilliSec());

    //返回lane给状态机做进一步处理
    res.traj = convertToMpcMsgsLane(waypoints);
    res.is_success = true;
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

    res.is_success = false;
  }
  return true;
}

inline bool HybridAstarNode::isSuccess(const SearchStatus &status)
{
  return status == SearchStatus::SUCCESS;
}

mpc_msgs::Lane HybridAstarNode::convertToMpcMsgsLane(const TrajectoryWaypoints &traj)
{
  mpc_msgs::Lane suv_lane;
  suv_lane.header = traj.header;

  for (int i = 0; i < traj.trajectory.size(); i++)
  {

    mpc_msgs::Waypoint temp_wp;
    geometry_msgs::PoseStamped temp_pose = traj.trajectory[i].pose;

    //转换到map坐标系下，给mpc追踪用
    const auto temp_pose_in_mapframe = transformPose(temp_pose.pose,
                                                     getTransform("map", temp_pose.header.frame_id));

    temp_pose.pose = temp_pose_in_mapframe;
    temp_pose.header.frame_id = "map";
    temp_pose.header.seq = i;
    temp_pose.header.stamp = ros::Time::now();

    temp_wp.pose = temp_pose;

    geometry_msgs::TwistStamped temp_twist;
    temp_twist.twist.linear.x = (i == traj.trajectory.size() - 1) ? 0.0 : _waypoints_velocity;
    temp_wp.twist = temp_twist;

    if (i != traj.trajectory.size() - 1 &&
        traj.trajectory[i].is_back != traj.trajectory[i + 1].is_back)
    {
      temp_wp.direction = 6;
      // ROS_INFO("stop");
    }
    else if (traj.trajectory[i].is_back == false)
    {
      temp_wp.direction = 0;
      // ROS_INFO("staright");
    }
    else if (traj.trajectory[i].is_back == true)
    {
      temp_wp.direction = 3;
      // ROS_INFO("back");
    }

    suv_lane.waypoints.push_back(temp_wp);
  }
  return suv_lane;
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
  if (clear_index == 0)
  {
    visualization_msgs::Marker vehicle_marker;
    vehicle_marker.header.frame_id = "map";
    vehicle_marker.header.stamp = ros::Time::now();
    vehicle_marker.id = id++;
    vehicle_marker.action = 3;

    _path_vehicles.markers.push_back(vehicle_marker);

    clear_index = 1;
  }

  for (int i = 0; i < visual_waypoints.trajectory.size(); i++)
  {
    visualization_msgs::Marker vehicle_marker;

    vehicle_marker.header.frame_id = "map";
    vehicle_marker.header.stamp = ros::Time::now();
    vehicle_marker.id = id++;
    vehicle_marker.type = visualization_msgs::Marker::CUBE;

    vehicle_marker.scale.x = _hybrid_astar_param.vehiche_shape.length;
    vehicle_marker.scale.y = _hybrid_astar_param.vehiche_shape.width;
    vehicle_marker.scale.z = 1.0;

    vehicle_marker.color.a = 0.2;

    if (i != visual_waypoints.trajectory.size() - 1 &&
        visual_waypoints.trajectory[i].is_back != visual_waypoints.trajectory[i + 1].is_back)
    {
      vehicle_marker.color.r = blue.red;
      vehicle_marker.color.g = blue.green;
      vehicle_marker.color.b = blue.blue;
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

    const auto vis_pose_in_mapframe = transformPose(visual_waypoints.trajectory[i].pose.pose,
                                                    getTransform("map", _costmap.header.frame_id));

    vehicle_marker.pose = vis_pose_in_mapframe;
    _path_vehicles.markers.push_back(vehicle_marker);
  }

  _pub_path_vehicles.publish(_path_vehicles);
}

void HybridAstarNode::run()
{
  ros::Rate rate(_update_rate);

  while (ros::ok())
  {
    ros::spinOnce();

    rate.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "astar_navi");

  HybridAstarNode node;
  node.run();

  return 0;
}
