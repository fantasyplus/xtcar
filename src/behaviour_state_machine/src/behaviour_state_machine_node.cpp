#include "behaviour_state_machine_node.h"

int BehaviourStateMachine::last_collision_car_index = 0;

BehaviourStateMachine::BehaviourStateMachine() : _nh(""), _private_nh("~")
{
    _private_nh.param<double>("sub_goal_tolerance_distance", _sub_goal_tolerance_distance, 30.0);
    _private_nh.param<int>("normal_zero_vel_segment", _normal_zero_vel_segment, 5);
    _private_nh.param<int>("collision_zero_vel_segment", _collision_zero_vel_segment, 1);
    _private_nh.param<bool>("use_complex_lane", use_complex_lane, false);
    _private_nh.param<double>("first_horizontal_distance", first_horizontal_distance, 2.0);
    _private_nh.param<double>("second_vertical_distance", second_vertical_distance, 10.0);
    _private_nh.param<double>("third_nearby_distance", third_nearby_distance, 5.0);
    _private_nh.param<double>("vehicle_length", vehicle_length, 4.788);
    _private_nh.param<double>("vehicle_width", vehicle_width, 2.198);
    _private_nh.param<double>("vehicle_cg2back", vehicle_cg2back, 1.367);
    _private_nh.param<double>("lookahead_distance", lookahead_distance, 5.0);
    _private_nh.param<double>("waypoints_velocity", waypoints_velocity, 2.0);
    _private_nh.param<int>("mode", mode, 1);
    _private_nh.param<double>("loop_rate", loop_rate, 100);
    _private_nh.param<std::string>("read_file_path", read_file_path, "/home/ros/fantasyplus/xtcar/src/mpc/traj/mpc_traj.txt");
    _private_nh.param<std::string>("save_file_path", save_file_path, "/home/ros/fantasyplus/xtcar/src/mpc/traj/mpc_traj.txt");

    /*---------------------subscriber---------------------*/
    _sub_costmap = _nh.subscribe("global_cost_map", 1, &BehaviourStateMachine::callbackCostMap, this);
    _sub_goal_pose = _nh.subscribe("move_base_simple/goal", 1, &BehaviourStateMachine::callbackGoalPose, this);
    _sub_rviz_start_pose = _nh.subscribe("initialpose", 1, &BehaviourStateMachine::callbackRvizStartPose, this);
    _sub_current_pose = _nh.subscribe("gnss_pose", 1, &BehaviourStateMachine::callbackCurrentPose, this);
    _sub_vehicle_status = _nh.subscribe("vehicle_status", 1, &BehaviourStateMachine::callbackVehicleStatus, this);
    _sub_scenario_mode = _nh.subscribe("scenario_mode", 1, &BehaviourStateMachine::callbackScenarioMode, this);
    _sub_task_status = _nh.subscribe("task_status", 1, &BehaviourStateMachine::callbackTaskStatus, this);

    /*---------------------advertiser---------------------*/
    _pub_mpc_lane = _nh.advertise<mpc_msgs::Lane>("mpc_waypoints", 1, false);
    _pub_vis_mpc_lane = _nh.advertise<nav_msgs::Path>("vis_mpc_waypoints", 1, false);
    // debug 可视化车辆轮廓
    _pub_vis_car_path = _nh.advertise<nav_msgs::Path>("vis_car_path", 1, false);
    _pub_task_control = _nh.advertise<mpc_msgs::TaskControl>("task_control", 1, false);

    /*---------------------timer---------------------*/
    double dt = 1 / loop_rate;
    //实时检测障碍物避碰且发布mpclane的定时器
    _timer_pub_lane = _nh.createTimer(ros::Duration(dt), &BehaviourStateMachine::callbackTimerPublishMpcLane, this);
    //静态地图测试的定时器
    _timer_static_exec = _nh.createTimer(ros::Duration(dt), &BehaviourStateMachine::callbackTimerStaticExec, this);
    //多段轨迹生成状态机的定时器
    _timer_multi_traj = _nh.createTimer(ros::Duration(dt), &BehaviourStateMachine::callbackTimerMultiTrajPlanning, this);
    //跟随固定轨迹的定时器
    _timer_path_tracing = _nh.createTimer(ros::Duration(dt), &BehaviourStateMachine::callbackTImerPathTracing, this);

    /*---------------------serviceClient---------------------*/
    _goal_pose_client = _nh.serviceClient<behaviour_state_machine::GoalPose>("goal_pose_srv");

    _tf_buffer = std::make_shared<tf2_ros::Buffer>();
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
}

geometry_msgs::Pose BehaviourStateMachine::transformPose(const geometry_msgs::Pose &pose,
                                                         const geometry_msgs::TransformStamped &transform)
{
    geometry_msgs::PoseStamped transformed_pose;
    geometry_msgs::PoseStamped orig_pose;
    orig_pose.pose = pose;
    tf2::doTransform(orig_pose, transformed_pose, transform);

    return transformed_pose.pose;
}

geometry_msgs::TransformStamped BehaviourStateMachine::getTransform(const std::string &target,
                                                                    const std::string &source)
{
    geometry_msgs::TransformStamped tf;
    try
    {
        tf = _tf_buffer->lookupTransform(target, source, ros::Time(0), ros::Duration(1));
    }
    catch (const tf2::LookupException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    return tf;
}

geometry_msgs::Pose BehaviourStateMachine::local2global(const nav_msgs::OccupancyGrid &costmap, const geometry_msgs::Pose &pose_local)
{
    tf2::Transform tf_origin;
    tf2::convert(costmap.info.origin, tf_origin);

    geometry_msgs::TransformStamped transform;
    transform.transform = tf2::toMsg(tf_origin);

    return transformPose(pose_local, transform);
}

geometry_msgs::Pose BehaviourStateMachine::global2local(const nav_msgs::OccupancyGrid &costmap, const geometry_msgs::Pose &pose_global)
{
    tf2::Transform tf_origin;
    tf2::convert(costmap.info.origin, tf_origin);

    geometry_msgs::TransformStamped transform;
    transform.transform = tf2::toMsg(tf_origin.inverse());

    return transformPose(pose_global, transform);
}

void BehaviourStateMachine::callbackTaskStatus(const mpc_msgs::TaskStatus &msg)
{
    task_status = msg;
}

void BehaviourStateMachine::callbackScenarioMode(const std_msgs::Int8 &msg)
{
    mode = static_cast<int>(msg.data) - 0x30;
}

void BehaviourStateMachine::callbackCostMap(const nav_msgs::OccupancyGridConstPtr &msg)
{
    //存储到全局代价地图中
    _cost_map_ptr = msg;
}

void BehaviourStateMachine::callbackCurrentPose(const geometry_msgs::PoseStamped &msg)
{
    _current_pose_flag = true;

    _current_pose_stamped = msg;
}

void BehaviourStateMachine::callbackVehicleStatus(const mpc_msgs::VehicleStatus &msg)
{
    _vehicle_status = msg;
}

void BehaviourStateMachine::callbackRvizStartPose(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
    _rviz_start_pose_stamped.header = msg.header;
    _rviz_start_pose_stamped.pose = msg.pose.pose;
}

void BehaviourStateMachine::callbackGoalPose(const geometry_msgs::PoseStamped &msg)
{
    _goal_pose_flag = true;

    id++;
    dynamic_id = 0;         //重新赋值为0，开始新的3段轨迹规划
    dynamic_complex_id = 0; //复杂轨迹处理专用判断是否接受到大目标点
    _goal_pose_stamped = msg;

    //每次接受到新的目标点才重新生成三段轨迹
    if (_current_pose_flag)
    {
        dir = getDirection(_current_pose_stamped, _goal_pose_stamped);
        sub_goal_vec = multipleTargetGenerator(dir);
    }
}

void BehaviourStateMachine::getClosestIndex(const mpc_msgs::Lane &temp_lane, int &closest_index)
{
    double min_dis = std::numeric_limits<int>::max();
    geometry_msgs::Pose cur_pose = _current_pose_stamped.pose;
    for (std::size_t i = 0; i < temp_lane.waypoints.size(); i++)
    {
        geometry_msgs::Pose temp_pose = temp_lane.waypoints[i].pose.pose;
        double dis = std::hypot(temp_pose.position.x - cur_pose.position.x, temp_pose.position.y - cur_pose.position.y);
        if (dis < min_dis)
        {
            closest_index = i;
            min_dis = dis;
        }
    }
}

void BehaviourStateMachine::getCollisionPoseVec(int closest_index,
                                                const mpc_msgs::Lane &temp_lane,
                                                std::vector<std::pair<geometry_msgs::Pose, double>> &base_pose_vec)
{
    double length = 0.0;

    //最近点先加进数组
    geometry_msgs::Pose pre_pose = temp_lane.waypoints[closest_index].pose.pose;
    base_pose_vec.push_back(std::pair<geometry_msgs::Pose, double>(pre_pose, length));

    for (std::size_t i = closest_index + 1; i < temp_lane.waypoints.size(); i++)
    {
        //超过前探距离了，退出循环
        if (length >= lookahead_distance)
        {
            break;
        }

        geometry_msgs::Pose temp_pose = temp_lane.waypoints[i].pose.pose;
        length += std::hypot(temp_pose.position.x - pre_pose.position.x, temp_pose.position.y - pre_pose.position.y);
        pre_pose = temp_pose;

        //最近点之后的点依次加进数组
        base_pose_vec.push_back(std::pair<geometry_msgs::Pose, double>(temp_pose, length));
    }
    //如果前探距离比路径还要长，实际上就是把最近点后面的所有点都加进数组了
}

void BehaviourStateMachine::computeCollisionIndexVec(const geometry_msgs::Pose &base_pose,
                                                     std::vector<std::pair<int, int>> &collision_index_vec,
                                                     nav_msgs::Path &vis_car_path)
{
    // debug 可视化车辆轮廓
    std::vector<geometry_msgs::PoseStamped> vis_car_pose;

    //获取当前点角度下的车体轮廓位置数组
    double cur_theta = tf2::getYaw(base_pose.orientation);
    std::vector<geometry_msgs::Pose> car_pose_vec;

    //车体参数
    const double back = -1.0 * vehicle_cg2back;
    const double front = vehicle_length - vehicle_cg2back;
    const double right = -1.0 * vehicle_width / 2;
    const double left = vehicle_width / 2;
    for (auto x = back; x <= front; x += _cost_map_ptr->info.resolution)
    {
        for (auto y = right; y < left; y += _cost_map_ptr->info.resolution)
        {
            const double offset_x = x * std::cos(cur_theta) - y * std::sin(cur_theta);
            const double offset_y = x * std::sin(cur_theta) + y * std::cos(cur_theta);

            // debug 可视化车体轮廓
            geometry_msgs::PoseStamped vis_pose;
            vis_pose.header.frame_id = "map";
            vis_pose.pose.position.x = base_pose.position.x + offset_x;
            vis_pose.pose.position.y = base_pose.position.y + offset_y;
            vis_pose.pose.position.z = base_pose.position.z;
            vis_car_pose.push_back(vis_pose);

            //车体轮廓点
            geometry_msgs::Pose car_pose;
            car_pose.position.x = base_pose.position.x + offset_x;
            car_pose.position.y = base_pose.position.y + offset_y;
            car_pose.position.z = base_pose.position.z;

            const auto car_pose_in_costmap_frame = transformPose(car_pose, getTransform(_cost_map_ptr->header.frame_id, "map"));
            car_pose_vec.push_back(global2local(*_cost_map_ptr, car_pose_in_costmap_frame));
        }
    }

    //转换车体轮廓点到栅格地图索引
    for (auto p : car_pose_vec)
    {
        std::pair<int, int> car_index_xy;
        car_index_xy.first = static_cast<int>(std::floor(p.position.x / _cost_map_ptr->info.resolution));
        car_index_xy.second = static_cast<int>(std::floor(p.position.y / _cost_map_ptr->info.resolution));
        collision_index_vec.push_back(car_index_xy);
    }

    // debug 可视化车辆轮廓
    for (auto sp : vis_car_pose)
    {
        vis_car_path.poses.push_back(sp);
    }
}

void BehaviourStateMachine::processMpcLane(mpc_msgs::Lane &mpc_lane, int start, int end, int zero_vel_segment, bool is_coll)
{
    //到这里的_mpc_lane应该都是单段轨迹，即只前进或只后退
    //对后1/zero_vel_segment的路径点速度赋0
    int size = end - start + 1;

    if (!is_coll) //无障碍物
    {
        //如果mpc路径比速度分段的分母还要小，路径最后一个点的速度置为0
        if ((int)mpc_lane.waypoints.size() >= zero_vel_segment)
        {
            mpc_lane.waypoints[mpc_lane.waypoints.size() - 1].twist.twist.linear.x = 0.0;
            return;
        }

        //获得开始赋0的起始点位置
        int zero_point_start_index = start + (size - (int)(((double)size / (double)zero_vel_segment) + 0.5)); //四舍五入
        for (int i = zero_point_start_index; i <= end; i++)
        {
            mpc_lane.waypoints[i].twist.twist.linear.x = 0.0;
        }
    }
    else //有障碍物
    {
        //获得开始赋0的起始点位置
        int zero_point_start_index = start + (size - (int)(((double)size / (double)zero_vel_segment)));
        for (int i = zero_point_start_index; i <= end; i++)
        {
            mpc_lane.waypoints[i].twist.twist.linear.x = 0.0;
        }
    }
}

void BehaviourStateMachine::callbackTimerPublishMpcLane(const ros::TimerEvent &e)
{
    /*-----实时检测当前路径上是否有障碍物，有则停车-----*/

    //获得当前正在执行的路径
    mpc_msgs::Lane temp_lane_to_detect_collision = _mpc_lane;
    if (temp_lane_to_detect_collision.waypoints.empty())
    {
        return;
    }

    //获取当前位置在路径上的最近点
    int closest_index = -1;
    getClosestIndex(temp_lane_to_detect_collision, closest_index);

    //获得从最近点往前看lookahead_distance距离的点序列,pair存储点和离最近点的距离
    //里面的每个点都用来检测碰撞
    std::vector<std::pair<geometry_msgs::Pose, double>> base_pose_vec;
    getCollisionPoseVec(closest_index, temp_lane_to_detect_collision, base_pose_vec);

    // debug 可视化车辆轮廓
    vis_car_path.poses.clear();

    //遍历每个点
    bool is_collision = false;
    double collision_length_to_cur_pose = 0.0;
    for (std::size_t i = 0; i < base_pose_vec.size(); i++)
    {
        geometry_msgs::Pose base_pose = base_pose_vec[i].first;
        double point_length_to_cur_pose = base_pose_vec[i].second;

        //计算当前点在栅格地图下的碰撞索引数组
        std::vector<std::pair<int, int>> collision_index_vec;
        computeCollisionIndexVec(base_pose, collision_index_vec, vis_car_path);

        //判断在当前路径点是否会发生碰撞
        for (auto car_index : collision_index_vec)
        {
            uint32_t coll_x_index = (uint32_t)car_index.first;
            uint32_t coll_y_index = (uint32_t)car_index.second;

            //不用做越界判断，因为混合A*初始路径就做过了越界判断
            if (_cost_map_ptr->data[coll_y_index * _cost_map_ptr->info.width + coll_x_index])
            {
                //碰撞

                //计算具体的碰撞点离当前位置的距离
                double coll_x = static_cast<double>(coll_x_index + 0.5) * _cost_map_ptr->info.resolution;
                double coll_y = static_cast<double>(coll_y_index + 0.5) * _cost_map_ptr->info.resolution;
                geometry_msgs::Pose coll_pose;
                coll_pose.position.x = coll_x;
                coll_pose.position.y = coll_y;
                coll_pose = local2global(*_cost_map_ptr, coll_pose);
                const auto coll_pose_in_map_frame = transformPose(coll_pose, getTransform("map", _cost_map_ptr->header.frame_id));
                collision_length_to_cur_pose = point_length_to_cur_pose + std::hypot(coll_pose_in_map_frame.position.x - base_pose.position.x, coll_pose_in_map_frame.position.y - base_pose.position.y);

                is_collision = true;
                last_collision_car_index = closest_index;

                //发生碰撞就跳出
                break;
            }
        }
        if (is_collision)
        {
            break;
        }
    }

    /*-----对mpclane进行处理,并发送真实路径-----*/

    if (is_collision) //重定义整条路径
    {
        ROS_INFO("collision at length_to_cur_pose:%f[m]", collision_length_to_cur_pose);
        //整条路径速度全部赋0
        processMpcLane(temp_lane_to_detect_collision, 0, temp_lane_to_detect_collision.waypoints.size() - 1, _collision_zero_vel_segment, true);
    }
    else //重定义当前位置到终点的路径
    {
        for (auto &wp : temp_lane_to_detect_collision.waypoints)
        {
            wp.twist.twist.linear.x = waypoints_velocity;
        }
        //最后一次前探距离中发生碰撞的时候的点到终点中的路径，1/_normal_zero_vel_segment的速度赋0
        // last_collision_car_index在每段轨迹中默认是0，即如果一次障碍物检测都没有发生的话，则一直发送最原始的路径（而不是从检测到障碍物从当前位置裁剪后的）
        processMpcLane(temp_lane_to_detect_collision, last_collision_car_index, temp_lane_to_detect_collision.waypoints.size() - 1, _normal_zero_vel_segment, false);
    }
    _pub_mpc_lane.publish(temp_lane_to_detect_collision);

    //保存当前路径作为文件
    if (dynamic_id != 4) //三段轨迹规划完之后就不用保存了
        saveTrajFile(temp_lane_to_detect_collision);

    // debug 可视化车辆轮廓
    vis_car_path.header.frame_id = "map";

    // if (dynamic_id != 4)
    _pub_vis_car_path.publish(vis_car_path);

    /*-----发送可视化路径-----*/
    nav_msgs::Path vis_lane;
    vis_lane.header.frame_id = "map";
    vis_lane.header.stamp = ros::Time::now();

    // mpc里的点本来就是在map下的
    for (std::size_t i = 0; i < temp_lane_to_detect_collision.waypoints.size(); i++)
    {
        geometry_msgs::PoseStamped temp_pose;
        temp_pose = temp_lane_to_detect_collision.waypoints[i].pose;

        vis_lane.poses.push_back(temp_pose);
    }

    // if (dynamic_id != 4)
    _pub_vis_mpc_lane.publish(vis_lane);
}

void BehaviourStateMachine::checkIsComplexLaneAndPrase(mpc_msgs::Lane &temp_lane, std::vector<mpc_msgs::Lane> &sub_lane_vec)
{
    sub_lane_vec.clear();

    // ROS_INFO("temp_lane.waypoints.size():%d", (int)temp_lane.waypoints.size());

    std::vector<int> turn_points_end_index;
    for (std::size_t i = 0; i < temp_lane.waypoints.size() - 1; i++)
    {
        if (temp_lane.waypoints[i].direction == 6)
        {
            turn_points_end_index.push_back(i);
        }
    }
    //如果是单向路径，不会有direction=6(停止位)的情况，直接返回
    if (turn_points_end_index.empty())
    {
        ROS_INFO("get single path");
        return;
    }
    else
    {
        ROS_INFO("processing complex path");
        is_complex_lane = true;

        int pre_end_index = 0;
        mpc_msgs::Lane sub_lane;
        for (int end_index : turn_points_end_index)
        {
            sub_lane.waypoints.assign(temp_lane.waypoints.begin() + pre_end_index, temp_lane.waypoints.begin() + end_index);
            // ROS_INFO("sub_lane.waypoints.size():%d", (int)sub_lane.waypoints.size());

            sub_lane_vec.push_back(sub_lane);

            pre_end_index = end_index;
        }
        //最后一段
        sub_lane.waypoints.assign(temp_lane.waypoints.begin() + pre_end_index, temp_lane.waypoints.end());
        // ROS_INFO("sub_lane.waypoints.size():%d", (int)sub_lane.waypoints.size());

        sub_lane_vec.push_back(sub_lane);
    }

    return;
}

double BehaviourStateMachine::getDistance(geometry_msgs::PoseStamped &p1, geometry_msgs::PoseStamped &p2)
{
    //转换p2到p1的坐标系下
    geometry_msgs::Pose p1_in_p2frame = transformPose(p1.pose, getTransform(_current_pose_stamped.header.frame_id, p1.header.frame_id));
    p1.pose = p1_in_p2frame;
    p1.header.frame_id = p1.header.frame_id;

    return std::hypot(p2.pose.position.x - p1.pose.position.x, p2.pose.position.y - p1.pose.position.y);
}

void BehaviourStateMachine::sendGoalSrv(geometry_msgs::PoseStamped &pose)
{
    _goal_pose_srv.request.pose = pose.pose;
    _goal_pose_srv.request.header = pose.header;

    if (_goal_pose_client.call(_goal_pose_srv))
    {
        bool is_success = _goal_pose_srv.response.is_success;
        if (is_success)
        {
            ROS_INFO("Hybrid Astar Successful");
        }
        else
        {
            ROS_INFO("Hybrid Astar Failed");
        }
    }
}

bool BehaviourStateMachine::sendGoalSrv(geometry_msgs::PoseStamped &pose, mpc_msgs::Lane &mpc_lane)
{
    _goal_pose_srv.request.pose = pose.pose;
    _goal_pose_srv.request.header = pose.header;

    mpc_msgs::Lane temp_lane;
    if (_goal_pose_client.call(_goal_pose_srv))
    {
        bool is_success = _goal_pose_srv.response.is_success;
        if (is_success)
        {
            ROS_INFO("Hybrid Astar Successful");
            temp_lane = _goal_pose_srv.response.traj;
            mpc_lane = temp_lane;

            return true;
        }
        else
        {
            ROS_INFO("Hybrid Astar Failed");
            return false;
        }
    }
    return false;
}

Direction BehaviourStateMachine::getDirection(geometry_msgs::PoseStamped &cur, geometry_msgs::PoseStamped &goal)
{
    geometry_msgs::TransformStamped goal2cur_tf;
    goal2cur_tf = getTransform("base_link", goal.header.frame_id);

    geometry_msgs::Pose pose_in_cur_frame = transformPose(goal.pose, goal2cur_tf);

    //(-90,0),(0,90)为正方向，(90,180),(-180,-90)为负方向
    double theta_in_cur_frame = tf2::getYaw(pose_in_cur_frame.orientation);
    theta_in_cur_frame = theta_in_cur_frame * 180.0 / M_PI;
    // ROS_INFO("theta_in_cur_frame:%f", theta_in_cur_frame);

    if (pose_in_cur_frame.position.y > 0)
    {
        if (theta_in_cur_frame >= -90.0 && theta_in_cur_frame <= 90.0)
        {
            ROS_INFO("Target at Current's Left and Orientation at Current's Forward");
            return Direction::ForwardLeft;
        }
        else if ((theta_in_cur_frame > 90.0 && theta_in_cur_frame <= 180.0) || (theta_in_cur_frame >= -180.0 && theta_in_cur_frame < -90.0))
        {
            ROS_INFO("Target at Current's Left and Orientation at Current's Back");
            return Direction::BackLeft;
        }
    }
    else if (pose_in_cur_frame.position.y < 0)
    {
        if (theta_in_cur_frame >= -90.0 && theta_in_cur_frame <= 90.0)
        {
            ROS_INFO("Target at Current's Right and Orientation at Current's Forward");
            return Direction::ForwardRight;
        }
        else if ((theta_in_cur_frame > 90.0 && theta_in_cur_frame <= 180.0) || (theta_in_cur_frame >= -180.0 && theta_in_cur_frame < -90.0))
        {
            ROS_INFO("Target at Current's Right and Orientation at Current's Back");
            return Direction::BackRight;
        }
    }

    return Direction::None;
}

std::vector<geometry_msgs::PoseStamped> BehaviourStateMachine::multipleTargetGenerator(Direction &dir)
{
    std::vector<double> dx;
    std::vector<double> dy;
    switch (dir)
    {
    //右前前(在车辆左边同方向)
    case Direction::ForwardLeft:
    {
        std::vector<double> left_dx{0, second_vertical_distance, third_nearby_distance};
        std::vector<double> left_dy{-first_horizontal_distance, 0, 0};

        dx.insert(dx.begin(), left_dx.begin(), left_dx.end());
        dy.insert(dy.begin(), left_dy.begin(), left_dy.end());

        break;
    }
    //左前前(在车辆左边反方向)
    case Direction::BackLeft:
    {
        std::vector<double> left_dx{0, second_vertical_distance, third_nearby_distance};
        std::vector<double> left_dy{first_horizontal_distance, 0, 0};

        dx.insert(dx.begin(), left_dx.begin(), left_dx.end());
        dy.insert(dy.begin(), left_dy.begin(), left_dy.end());

        break;
    }
    //左前前(在车辆右边同方向)
    case Direction::ForwardRight:
    {
        std::vector<double> right_dx{0, second_vertical_distance, third_nearby_distance};
        std::vector<double> right_dy{first_horizontal_distance, 0, 0};

        dx.insert(dx.begin(), right_dx.begin(), right_dx.end());
        dy.insert(dy.begin(), right_dy.begin(), right_dy.end());

        break;
    }
    //右前前(在车辆右边反方向)
    case Direction::BackRight:
    {
        std::vector<double> right_dx{0, second_vertical_distance, third_nearby_distance};
        std::vector<double> right_dy{-first_horizontal_distance, 0, 0};

        dx.insert(dx.begin(), right_dx.begin(), right_dx.end());
        dy.insert(dy.begin(), right_dy.begin(), right_dy.end());

        break;
    }
    default:
        break;
    }

    std::vector<geometry_msgs::PoseStamped> sub_goal_vec;

    for (std::size_t i = 0; i < dx.size(); i++)
    {
        geometry_msgs::PoseStamped pre_sub_goal;

        pre_sub_goal.header.seq = _goal_pose_stamped.header.seq;
        pre_sub_goal.header.stamp = _goal_pose_stamped.header.stamp;
        pre_sub_goal.header.frame_id = "map"; //最终是要转换成map下的坐标
        pre_sub_goal.pose = _goal_pose_stamped.pose;

        double theta = tf2::getYaw(_goal_pose_stamped.pose.orientation);
        pre_sub_goal.pose.position.x += dx[i] * std::cos(theta) - dy[i] * std::sin(theta);
        pre_sub_goal.pose.position.y += dx[i] * std::sin(theta) + dy[i] * std::cos(theta);

        // ROS_INFO("_costmap_frame_id:%s , _goal_pose_stamped:%s", _costmap_frame_id.c_str(), _goal_pose_stamped.header.frame_id.c_str());
        auto _target_tf = getTransform("map", _goal_pose_stamped.header.frame_id);

        auto pose_in_costmap_frame = transformPose(pre_sub_goal.pose, _target_tf);

        // ROS_INFO("pose_in_costmap_frame:%f,%f", pose_in_costmap_frame.position.x, pose_in_costmap_frame.position.y);

        pre_sub_goal.pose = pose_in_costmap_frame;

        sub_goal_vec.push_back(pre_sub_goal);
    }

    return sub_goal_vec;
}

void BehaviourStateMachine::callbackTimerStaticExec(const ros::TimerEvent &e)
{
    if (mode != (int)ScenarioStatus::StaticExec)
    {
        return;
    }
    //每次接受到新的目标点才发送srv
    if (id != pre_id)
    {
        pre_id = id;

        //实验用
        //保留点 ratio=0.5 319ms opennode:2692
        //不加thetacost 2066ms opennode:2094
        // geometry_msgs::PoseWithCovarianceStamped temp_start;
        // temp_start.header.stamp = ros::Time::now();
        // temp_start.header.frame_id = "map";
        // temp_start.pose.pose.position.x = 27.391078949;
        // temp_start.pose.pose.position.y = 18.884557724;
        // temp_start.pose.pose.position.z = 0.0;
        // temp_start.pose.pose.orientation.x = 0.0;
        // temp_start.pose.pose.orientation.y = 0.0;
        // temp_start.pose.pose.orientation.z = 1.0;
        // temp_start.pose.pose.orientation.w = -4.37113900019e-08;
        // _pub_rviz_start_pose.publish(temp_start);

        // _goal_pose_stamped.header.stamp = ros::Time::now();
        // _goal_pose_stamped.header.frame_id = "map";
        // _goal_pose_stamped.pose.position.x = 29.698841095;
        // _goal_pose_stamped.pose.position.y = 77.3743591309;
        // _goal_pose_stamped.pose.position.z = 0.0;
        // _goal_pose_stamped.pose.orientation.x = 0.0;
        // _goal_pose_stamped.pose.orientation.y = 0.0;
        // _goal_pose_stamped.pose.orientation.z = 0.721826900025;
        // _goal_pose_stamped.pose.orientation.w = 0.692073642324;

        sendGoalSrv(_goal_pose_stamped);
    }
}

void BehaviourStateMachine::callbackTimerMultiTrajPlanning(const ros::TimerEvent &e)
{
    if (mode != (int)ScenarioStatus::MultiTrajPlanning)
    {
        return;
    }

    static geometry_msgs::PoseStamped pre_sub_goal;
    static std::vector<mpc_msgs::Lane> sub_lane_vec;

    if (dynamic_id != 4) //发送3段轨迹（子目标点）且车辆执行完毕后就停止，直到接受到新的大目标点
    {

        //如果是复杂轨迹(有前进有后退)，先不考虑正常的子目标点生成，对它单独处理
        //不处理完这段轨迹是不会进入后面的流程的
        if (is_complex_lane && use_complex_lane)
        {
            //如果复杂轨迹数组为空了，说明处理完该段轨迹了，进入正常的流程
            if (sub_lane_vec.empty())
            {
                is_complex_lane = false;
                dynamic_id++; //转移到下一个正常的子目标点(如果为3的话，其实就是跳转到最后的终点判断流程)
                return;
            }
            //如果接受到新的大目标点的了，重新进入正常流程判断
            if (dynamic_complex_id == 0)
            {
                is_complex_lane = false;
                return;
            }

            //每次发送复杂轨迹数组的第一个(即单个简单轨迹)，并且把轨迹最后的点视为终点并判断是否到达
            _mpc_lane = sub_lane_vec[0];

            //发送任务状态字给底层
            task_control.traj_end = 0;
            task_control.traj_turn = 0;
            _pub_task_control.publish(task_control);

            int sub_turn_index = sub_lane_vec[0].waypoints.size() - 1;
            geometry_msgs::PoseStamped pre_complex_sub_goal; //当前子轨迹的终点
            pre_complex_sub_goal = sub_lane_vec[0].waypoints[sub_turn_index].pose;

            double length = getDistance(_current_pose_stamped, pre_complex_sub_goal);
            // ROS_INFO("remain length:%f", length);

            if (length < _sub_goal_tolerance_distance && (_vehicle_status.speed == 0.0 || _vehicle_status.acc == 0.0)) //判断车辆是否到达前一段轨迹的终点
            {
                ROS_INFO("arrive at no.%d pre_complex_sub_goal", (int)sub_lane_vec.size());

                //发送任务状态字给底层
                task_control.traj_end = 0;
                task_control.traj_turn = 1;
                _pub_task_control.publish(task_control);

                //删除当前轨迹
                sub_lane_vec.erase(sub_lane_vec.begin());

                //下一段轨迹的"最后一次在前探距离上发生碰撞时车体在路径上的位置索引"初始化为0
                last_collision_car_index = 0;

                //直接进入下一次循环，避免在复杂轨迹处理完之前进入到正常流程
                return;
            }
        }

        //正常简单轨迹流程
        if (dynamic_id == 0 && !is_complex_lane) //第一段
        {
            ROS_INFO("---------------send No.%d sub_goal------------------", dynamic_id + 1);

            //发送任务状态字给底层
            task_control.traj_end = 0;
            task_control.traj_turn = 0;
            _pub_task_control.publish(task_control);

            pre_sub_goal = sub_goal_vec[dynamic_id]; //记录下第一段的子目标点

            mpc_msgs::Lane temp_lane;
            bool ret = sendGoalSrv(sub_goal_vec[dynamic_id], temp_lane); //发送第一个子目标点并将返回轨迹保存到temp_lane中

            //如果规划失败了，退出正常流程
            if (!ret)
            {
                dynamic_id = 4;
                return;
            }

            if (use_complex_lane)
            {
                checkIsComplexLaneAndPrase(temp_lane, sub_lane_vec);
            }

            //如果不是复杂轨迹，直接把整条轨迹发出去
            if (!is_complex_lane)
            {
                _mpc_lane = temp_lane;
                //下一段轨迹的"最后一次在前探距离上发生碰撞时车体在路径上的位置索引"初始化为0
                last_collision_car_index = 0;
                dynamic_id++;
            }
            else
            {
                ROS_INFO("have %d complex sub goal", (int)sub_lane_vec.size());
                dynamic_complex_id = 1;
                return;
            }
        }
        else if (dynamic_id > 0 && !is_complex_lane) //剩下两段
        {

            //得到距离子目标点的长度(如果前一段是复杂轨迹的话，那么车辆应该已经到前一段的终点了)
            double length = getDistance(_current_pose_stamped, pre_sub_goal);
            // ROS_INFO("remain length:%f", length);

            if (length < _sub_goal_tolerance_distance && (_vehicle_status.speed == 0.0 || _vehicle_status.acc == 0.0)) //判断车辆是否到达前一段轨迹的终点
            {

                //发送任务状态字给底层
                task_control.traj_end = 0;
                task_control.traj_turn = 1;
                _pub_task_control.publish(task_control);

                //到达第三段轨迹的终点
                if (dynamic_id == 3)
                {
                    ROS_INFO("---------------enter multi traj end point!------------------");

                    //发送任务状态字给底层
                    task_control.traj_end = 1;
                    task_control.traj_turn = 0;
                    _pub_task_control.publish(task_control);

                    //++之后三段轨迹规划结束
                    dynamic_id++;

                    //分离出一个单独的线程，处理最后一段两车跟随的逻辑
                    task_status_flag = false;
                    std::thread send_last_traj(std::bind(&BehaviourStateMachine::threadSendLastTraj, this));
                    send_last_traj.detach();

                    return;
                }

                ROS_INFO("---------------send No.%d sub_goal------------------", dynamic_id + 1);
                pre_sub_goal = sub_goal_vec[dynamic_id]; //记录下新一段轨迹的子目标点

                mpc_msgs::Lane temp_lane;
                bool ret = sendGoalSrv(sub_goal_vec[dynamic_id], temp_lane); //发送子目标点并将返回轨迹保存到temp_lane中

                //如果规划失败了，退出正常流程
                if (!ret)
                {
                    dynamic_id = 4;
                    return;
                }

                if (use_complex_lane)
                {
                    checkIsComplexLaneAndPrase(temp_lane, sub_lane_vec);
                }

                //如果不是复杂轨迹，直接把整条轨迹发出去
                if (!is_complex_lane)
                {
                    _mpc_lane = temp_lane;
                    //下一段轨迹的最后一次在前探距离上发生碰撞时车体在路径上的位置索引初始化为0
                    last_collision_car_index = 0;
                    dynamic_id++;
                }
                else
                {
                    ROS_INFO("have %d complex sub goal", (int)sub_lane_vec.size());
                    dynamic_complex_id = 1;
                    return;
                }
            }
            else //没到的话就跳过
            {
                //发送任务状态字给底层
                task_control.traj_end = 0;
                task_control.traj_turn = 0;
                _pub_task_control.publish(task_control);
                return;
            }
        }
    }
    else
    {
        //重置静态变量，为下一次接受大目标点做准备
        geometry_msgs::PoseStamped empty_pose;
        pre_sub_goal = empty_pose;

        //重置复杂轨迹数组
        sub_lane_vec.clear();

        return;
    }
}

void BehaviourStateMachine::saveTrajFile(const mpc_msgs::Lane save_lane)
{
    std::ofstream out;
    out.open(save_file_path, std::ios::trunc | std::ios::out);
    if (out.fail())
    {
        ROS_WARN("save traj file failed");
        return;
    }

    for (std::size_t i = 0; i < save_lane.waypoints.size(); i++)
    {

        mpc_msgs::Waypoint waypoint_temp = save_lane.waypoints[i];
        out << waypoint_temp.pose.pose.position.x << ' '
            << waypoint_temp.pose.pose.position.y << ' '
            << waypoint_temp.pose.pose.position.z << ' '
            << waypoint_temp.pose.pose.orientation.x << ' '
            << waypoint_temp.pose.pose.orientation.y << ' '
            << waypoint_temp.pose.pose.orientation.z << ' '
            << waypoint_temp.pose.pose.orientation.w << ' '
            << waypoint_temp.direction << ' '
            << waypoint_temp.twist.twist.linear.x
            << std::endl;
    }

    out.close();

    // ROS_INFO("save traj size:%d", (int)save_lane.waypoints.size());
}

void BehaviourStateMachine::readTrajFile(mpc_msgs::Lane &send_lane)
{
    std::ifstream fin(read_file_path, std::ios::in);
    if (fin.fail())
    {
        send_lane.waypoints.clear();
        return;
    }

    ROS_INFO("open traj file successfully");

    mpc_msgs::Waypoint send_wp;

    std::vector<std::vector<double>> traj_datas;
    std::vector<double> single_wp;

    double input;
    while (fin >> input)
    {
        single_wp.push_back(input);

        if (fin.get() == '\n')
        {
            traj_datas.push_back(single_wp);
            single_wp.clear();
            continue;
        }
    }

    for (auto wp : traj_datas)
    {
        send_wp.pose.pose.position.x = wp[0];
        send_wp.pose.pose.position.y = wp[1];
        send_wp.pose.pose.position.z = wp[2];

        send_wp.pose.pose.orientation.x = wp[3];
        send_wp.pose.pose.orientation.y = wp[4];
        send_wp.pose.pose.orientation.z = wp[5];
        send_wp.pose.pose.orientation.w = wp[6];

        send_wp.direction = wp[7];
        send_wp.twist.twist.linear.x = wp[8];

        send_lane.waypoints.push_back(send_wp);
    }
}

void BehaviourStateMachine::threadSendLastTraj()
{
    //收到1代表小车和大车位姿细调完成，进入发送跟随轨迹阶段
    while (true)
    {

        // std::cout << std::this_thread::get_id() << std::endl;
        ROS_INFO("waiting car pose adjust completed......need task_end==1 , now task_end==%d", (int)task_status.task_end);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        //如果在跟随过程中重发大目标点了，直接结束当前跟随线程
        if (dynamic_id != 4)
        {
            ROS_WARN("end follow thread");
            return;
        }

        if ((int)task_status.task_end != 1 && !task_status_flag)
        {
            continue;
        }
        else
        {
            task_status_flag = true;
            break;
        }
    }

    mpc_msgs::Lane send_lane;
    readTrajFile(send_lane);
    if (send_lane.waypoints.empty())
    {
        ROS_ERROR("can't open traj_file in threadSendLastTraj()");
        return;
    }

    ROS_INFO("---------------send last follow lane------------------ , size:%d", (int)send_lane.waypoints.size());

    //发送最后一段固定轨迹
    _mpc_lane = send_lane;

    while (true)
    {

        // std::cout << std::this_thread::get_id() << std::endl;
        ROS_INFO("waiting two car follow lane completed......need task_end==2 , now task_end==%d", (int)task_status.task_end);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        //如果在跟随过程中重发大目标点了，直接结束当前跟随线程
        if (dynamic_id != 4)
        {
            ROS_WARN("end follow thread");
            return;
        }

        //收到2代表两车跟随轨迹完成，进入最终的结束阶段并结束该线程
        if ((int)task_status.task_end == 2)
        {
            ROS_INFO("---------------enter last traj end point---------------");
            //清空_mpc_lane
            _mpc_lane.waypoints.clear();

            //清空可视化的东西
            int cnt = 10;
            while (cnt--)
            {
                nav_msgs::Path empty_path;
                empty_path.header.frame_id = "map";
                _pub_vis_car_path.publish(empty_path);
                _pub_vis_mpc_lane.publish(empty_path);
            }

            return;
        }
    }
}

void BehaviourStateMachine::callbackTImerPathTracing(const ros::TimerEvent &e)
{
    if (mode != (int)ScenarioStatus::PathTracing)
    {
        return;
    }

    mpc_msgs::Lane send_lane;
    readTrajFile(send_lane);
    if (send_lane.waypoints.empty())
    {
        ROS_ERROR("can't open traj_file in callbackTImerPathTracing()");
        return;
    }
}

void BehaviourStateMachine::run()
{
    signal(SIGINT, MySigintHandler);

    ROS_INFO("--------Choose a mode first!-------");
    ROS_INFO("--------0:Stop-------");
    ROS_INFO("--------1:MultiTrajPlanning-------");
    ROS_INFO("--------2:PathTracing-------");
    ROS_INFO("--------3:StaticExec-------");

    ros::Rate rate(loop_rate);
    while (ros::ok())
    {
        ros::spinOnce();

        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "behaviour_state_machine");

    BehaviourStateMachine obj;
    obj.run();

    return 0;
}