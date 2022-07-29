#include "behaviour_state_machine_truck_node.h"

BehaviourStateMachine::BehaviourStateMachine() : _nh(""), _private_nh("~")
{
    /*---------------------通用参数---------------------*/
    _private_nh.param<bool>("use_gear", use_gear, false);
    _private_nh.param<std::string>("follow_file_path", follow_file_path, "/home/ros/fantasyplus/xtcar/src/mpc/traj/mpc_traj.txt");
    _private_nh.param<std::string>("save_file_path", save_file_path, "/home/ros/fantasyplus/xtcar/src/mpc/traj/mpc_traj.txt");
    _private_nh.param<double>("waypoints_velocity", waypoints_velocity, 2.0);
    _private_nh.param<double>("start_waypoints_velocity", start_waypoints_velocity, 0.5);
    _private_nh.param<double>("max_velo_ratio", max_velo_ratio, 2.0);
    _private_nh.param<double>("start_velo_ratio", start_velo_ratio, 1.6);
    _private_nh.param<int>("mode", mode, 2);
    _private_nh.param<double>("loop_rate", loop_rate, 100);

    /*---------------------前场参数---------------------*/
    _private_nh.param<double>("sub_goal_tolerance_distance", sub_goal_tolerance_distance, 30.0);
    _private_nh.param<bool>("use_complex_lane", use_complex_lane, false);
    _private_nh.param<double>("first_horizontal_distance", first_horizontal_distance, 2.0);
    _private_nh.param<double>("second_vertical_distance", second_vertical_distance, 10.0);
    _private_nh.param<double>("third_nearby_distance", third_nearby_distance, 5.0);

    /*---------------------避障参数---------------------*/
    _private_nh.param<double>("vehicle_length", vehicle_length, 4.788);
    _private_nh.param<double>("vehicle_width", vehicle_width, 2.198);
    _private_nh.param<double>("vehicle_cg2back", vehicle_cg2back, 1.367);
    _private_nh.param<double>("lookahead_distance", lookahead_distance, 5.0);

    /*---------------------后场参数---------------------*/
    _private_nh.param<double>("stop_distance", stop_distance, 5.0);
    _private_nh.param<double>("stop_theta", stop_theta, 5.0);
    _private_nh.param<bool>("is_show_debug", is_show_debug, true);
    _private_nh.param<int>("stop_time", stop_time, 5.0);
    _private_nh.param<int>("back_waypoints_num", back_waypoints_num, 10);
    _private_nh.param<int>("front_waypoints_num", front_waypoints_num, 50);
    _private_nh.param<int>("start_waypoints_num", start_waypoints_num, 10);

    _private_nh.param<std::string>("fixed_traj_path", fixed_traj_path, "/home/ros/fantasyplus/xtcar/src/mpc/traj/hc_mpc_traj.txt");

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
    //静态地图测试的定时器
    _timer_static_exec = _nh.createTimer(ros::Duration(dt), &BehaviourStateMachine::callbackTimerStaticExec, this);

    /*---------------------serviceClient---------------------*/
    _goal_pose_client = _nh.serviceClient<behaviour_state_machine_truck::GoalPose>("goal_pose_srv");

    _tf_buffer = std::make_shared<tf2_ros::Buffer>();
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

    //开启一个后台线程，专门用于发布与底层通信的topic
    std::thread send_status_topic = std::thread(std::bind(&BehaviourStateMachine::threadSendStatusTopic, this));
    send_status_topic.detach();

    //多段轨迹生成状态机的线程（前场模块）
    std::thread multi_traj_planning = std::thread(std::bind(&BehaviourStateMachine::threadMultiTrajPlanning, this));
    multi_traj_planning.detach();

    //实时检测障碍物避碰且发布mpclane的线程
    std::thread publish_mpc_lane = std::thread(std::bind(&BehaviourStateMachine::threadPublishMpcLane, this));
    publish_mpc_lane.detach();

    //开启一个后台线程，用于处理重卡跟随小车的逻辑（前场模块,临时）
    std::thread send_last_traj(std::bind(&BehaviourStateMachine::threadSendLastTraj, this));
    send_last_traj.detach();

    //重卡后场模块
    std::thread path_tracing_and_stop(std::bind(&BehaviourStateMachine::threadPathTracingAndStop, this));
    path_tracing_and_stop.detach();

    //保存最初的速度值
    prev_waypoints_velocity = waypoints_velocity;
    prev_start_waypoints_velocity = start_waypoints_velocity;
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

void BehaviourStateMachine::threadSendStatusTopic()
{
    while (true)
    {
        //控制帧率
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        _pub_task_control.publish(task_control);
    }
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
    is_get_costmap = true;
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

void BehaviourStateMachine::processMpcLane(mpc_msgs::Lane &cur_lane, int start, int end, bool is_stop, int closest_idx)
{
    //到这里的cur_lane应该都是单段轨迹，即只前进或只后退

    if (!is_stop) //无障碍物
    {
        //最近点之前的全部赋0
        for (int i = 0; i < closest_idx; i++)
        {
            cur_lane.waypoints[i].twist.twist.linear.x = 0.0;
        }

        //最近点往后start_waypoints_num个路径点速度为start_waypoints_velocity
        int start_end = std::min(start_waypoints_num + closest_idx, end);
        for (int i = closest_idx; i < start_end; i++)
        {
            cur_lane.waypoints[i].twist.twist.linear.x = start_waypoints_velocity;
        }

        //到start之前的点速度为waypoints_velocity
        for (int i = start_end; i < start; i++)
        {
            cur_lane.waypoints[i].twist.twist.linear.x = waypoints_velocity;
        }

        //再把start到end的点赋为0
        for (int i = start; i < end; i++)
        {
            cur_lane.waypoints[i].twist.twist.linear.x = 0.0;
        }
    }
    else if (is_stop) //有障碍物
    {
        //整条路径全部赋0
        for (auto &wp : cur_lane.waypoints)
        {
            wp.twist.twist.linear.x = 0.0;
        }
    }
}

void BehaviourStateMachine::checkCollision(bool &is_collision, double &collision_length_to_cur_pose,
                                           std::vector<std::pair<geometry_msgs::Pose, double>> &base_pose_vec)
{
    auto isOutOfRange = [this](const uint32_t coll_x_index, const uint32_t coll_y_index)
    {
        if (coll_x_index < 0 || coll_x_index >= _cost_map_ptr->info.width)
        {
            return true;
        }
        if (coll_y_index < 0 || coll_y_index >= _cost_map_ptr->info.height)
        {
            return true;
        }
        return false;
    };

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

            bool is_out_of_range = isOutOfRange(coll_x_index, coll_y_index);
            if (is_out_of_range)
            {
                //如果越界了，放弃判断碰撞
                // ROS_INFO("out of occupancygrid range!");

                is_collision = false;
                break;
            }

            //碰撞判断
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

                //发生碰撞就跳出
                break;
            }
        }
        if (is_collision)
        {
            break;
        }
    }
}

void BehaviourStateMachine::reshapeLane(const mpc_msgs::Lane &in_lane, mpc_msgs::Lane &out_lane, int closest_index)
{
    //加入最近点之前的back_waypoints_num个路径点
    int back_start = std::max(closest_index - back_waypoints_num, 0);
    for (int i = back_start; i < closest_index; i++)
    {
        out_lane.waypoints.push_back(in_lane.waypoints[i]);
        out_lane.waypoints.back().twist = in_lane.waypoints[closest_index].twist;
    }

    //加入最近点之后的front_waypoints_num个路径点
    int front_end = std::min(front_waypoints_num + closest_index, (int)in_lane.waypoints.size());
    for (int i = closest_index; i < front_end; i++)
    {
        out_lane.waypoints.push_back(in_lane.waypoints[i]);
    }
}

void BehaviourStateMachine::visualMpcLane(const mpc_msgs::Lane &send_lane)
{
    nav_msgs::Path vis_lane;
    vis_lane.header.frame_id = "map";
    vis_lane.header.stamp = ros::Time::now();

    // mpc里的点本来就是在map下的,不用做坐标变换了
    for (std::size_t i = 0; i < send_lane.waypoints.size(); i++)
    {
        geometry_msgs::PoseStamped temp_pose;
        temp_pose = send_lane.waypoints[i].pose;

        vis_lane.poses.push_back(temp_pose);
    }

    _pub_vis_mpc_lane.publish(vis_lane);
}

void BehaviourStateMachine::threadPublishMpcLane()
{
    while (true)
    {
        /*-----实时检测当前路径上是否有障碍物，有则停车-----*/

        //获得当前正在执行的路径
        //路径为空或者还没收到代价地图则不执行
        mpc_msgs::Lane cur_lane = _mpc_lane;
        if (cur_lane.waypoints.empty() || !is_get_costmap)
        {
            continue;
        }

        //获取当前位置在路径上的最近点
        int closest_index = -1;
        getClosestIndex(cur_lane, closest_index);

        //获得从最近点往前看lookahead_distance距离的点序列,pair存储点和离最近点的距离
        //里面的每个点都用来检测碰撞
        std::vector<std::pair<geometry_msgs::Pose, double>> base_pose_vec;
        getCollisionPoseVec(closest_index, cur_lane, base_pose_vec);

        // debug 可视化车辆轮廓
        vis_car_path.poses.clear();

        //遍历每个点,检查碰撞
        bool is_collision = false;
        double collision_length_to_cur_pose = 0.0;
        checkCollision(is_collision, collision_length_to_cur_pose, base_pose_vec);

        /*-----对mpclane进行处理,并发送真实路径-----*/

        if (is_collision || stop_flag) //有碰撞或者要停车
        {
            // ROS_INFO("collision at length_to_cur_pose:%f[m]", collision_length_to_cur_pose);
            //整条路径速度全部赋0
            processMpcLane(cur_lane, 0, cur_lane.waypoints.size(), true, 0);

            //更新这一次碰撞或者停车时的最近点信息
            last_stop_collision_car_index = closest_index;
        }
        else //无碰撞或不停车
        {
            //起步路径处理
            //若从头到尾都没有碰撞或停车，则从last_stop_collision_car_index在后场处理线程中初始的最近点开始
            //若发生了碰撞或停车，则从last_stop_collision_car_index更新的地方开始
            processMpcLane(cur_lane, cur_lane.waypoints.size() - 3, cur_lane.waypoints.size(), false, last_stop_collision_car_index);
        }
        //保存当前路径作为文件
        if (dynamic_id != 4) //三段轨迹规划完之后就不用保存了
            saveTrajFile(cur_lane);

        //裁剪路径
        mpc_msgs::Lane send_lane;
        reshapeLane(cur_lane, send_lane, closest_index);

        if (is_show_debug)
        {
            for (auto wp : send_lane.waypoints)
            {
                std::cout << wp.twist.twist.linear.x << " ";
            }
            std::cout << "end\n";
        }

        //发送最终的路径
        _pub_mpc_lane.publish(send_lane);

        // debug 可视化车辆轮廓
        vis_car_path.header.frame_id = "map";

        _pub_vis_car_path.publish(vis_car_path);

        //发送可视化路径
        visualMpcLane(send_lane);

        //控制帧率
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
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

void BehaviourStateMachine::threadMultiTrajPlanning()
{
    while (true)
    {
        //控制帧率
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        if (mode != (int)ScenarioStatus::MultiTrajPlanning)
        {
            continue;
        }

        static geometry_msgs::PoseStamped pre_sub_goal;
        static std::vector<mpc_msgs::Lane> sub_lane_vec;

        //在初始状态，一直返回不进入后面的流程
        if (task_control.traj_end == 0)
        {

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            if (dynamic_id == 0)
            {
                //收到大目标点，进入三段轨迹流程
                //发送任务状态字给底层(进入三段轨迹)
                task_control.traj_end = 1;

                continue;
            }

            ROS_INFO("---------------initial state------------------");

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

            continue;
        }

        if (dynamic_id != 4 && task_control.traj_end == 1) //发送3段轨迹（子目标点）且车辆执行完毕后就停止，直到接受到新的大目标点
        {

            //如果收到了重置信号，返回初始状态
            if ((int)task_status.task_error == 1)
            {
                //发送任务状态字给底层(直接返回初始状态)
                task_control.traj_end = 0;

                continue;
            }

            //如果是复杂轨迹(有前进有后退)，先不考虑正常的子目标点生成，对它单独处理
            //不处理完这段轨迹是不会进入后面的流程的
            if (is_complex_lane && use_complex_lane)
            {
                //如果复杂轨迹数组为空了，说明处理完该段轨迹了，进入正常的流程
                if (sub_lane_vec.empty())
                {
                    is_complex_lane = false;
                    dynamic_id++; //转移到下一个正常的子目标点(如果为3的话，其实就是跳转到最后的终点判断流程)
                    continue;
                }

                //如果接受到新的大目标点的了，重新进入正常流程判断
                if (dynamic_complex_id == 0)
                {
                    is_complex_lane = false;

                    continue;
                }

                //每次发送复杂轨迹数组的第一个(即单个简单轨迹)，并且把轨迹最后的点视为终点并判断是否到达
                _mpc_lane = sub_lane_vec[0];

                int sub_turn_index = sub_lane_vec[0].waypoints.size() - 1;
                geometry_msgs::PoseStamped pre_complex_sub_goal; //当前子轨迹的终点
                pre_complex_sub_goal = sub_lane_vec[0].waypoints[sub_turn_index].pose;

                double length = getDistance(_current_pose_stamped, pre_complex_sub_goal);
                // ROS_INFO("remain length:%f", length);

                if (length < sub_goal_tolerance_distance && (_vehicle_status.speed == 0.0 || _vehicle_status.acc == 0.0)) //判断车辆是否到达前一段轨迹的终点
                {
                    ROS_INFO("arrive at no.%d pre_complex_sub_goal", (int)sub_lane_vec.size());

                    //如果位置和速度满足子目标点要求，等待档位切换到P档再进行下一步
                    waitingCarChangeToParkingGear(use_gear);

                    //删除当前轨迹
                    sub_lane_vec.erase(sub_lane_vec.begin());

                    //直接进入下一次循环，避免在复杂轨迹处理完之前进入到正常流程
                    continue;
                }
            }

            //正常简单轨迹流程
            if (dynamic_id == 0 && !is_complex_lane) //第一段
            {
                ROS_INFO("---------------send No.%d sub_goal------------------", dynamic_id + 1);

                pre_sub_goal = sub_goal_vec[dynamic_id]; //记录下第一段的子目标点

                bool ret = false;
                mpc_msgs::Lane temp_lane;
                //尝试10次规划
                int retry_cnt = 10;
                while (retry_cnt--)
                {
                    ROS_INFO("---------------try %d times planning------------------", 10 - retry_cnt);

                    //发送第一个子目标点并将返回轨迹保存到temp_lane中
                    ret = sendGoalSrv(sub_goal_vec[dynamic_id], temp_lane);

                    //如果规划成功，退出
                    if (ret)
                    {
                        break;
                    }
                }

                //如果规划失败了，回到初始状态
                if (!ret)
                {
                    ROS_INFO("---------------Hybrid Astar Planning Failed------------------");

                    dynamic_id = 4;

                    //发送任务状态字给底层(直接返回初始状态)
                    task_control.traj_end = 0;

                    continue;
                }

                if (use_complex_lane)
                {
                    checkIsComplexLaneAndPrase(temp_lane, sub_lane_vec);
                }

                //如果不是复杂轨迹，直接把整条轨迹发出去
                if (!is_complex_lane)
                {
                    _mpc_lane = temp_lane;
                    dynamic_id++;
                }
                else
                {
                    ROS_INFO("have %d complex sub goal", (int)sub_lane_vec.size());
                    dynamic_complex_id = 1;
                    continue;
                }
            }
            else if (dynamic_id > 0 && !is_complex_lane) //剩下两段
            {

                //得到距离子目标点的长度(如果前一段是复杂轨迹的话，那么车辆应该已经到前一段的终点了)
                double length = getDistance(_current_pose_stamped, pre_sub_goal);
                // ROS_INFO("remain length:%f", length);

                if (length < sub_goal_tolerance_distance && (_vehicle_status.speed == 0.0 || _vehicle_status.acc == 0.0)) //判断车辆是否到达前一段轨迹的终点
                {
                    //如果位置和速度满足子目标点要求，等待档位切换到P档再进行下一步
                    waitingCarChangeToParkingGear(use_gear);

                    //到达第三段轨迹的终点
                    if (dynamic_id == 3)
                    {
                        ROS_INFO("---------------enter multi trajs end point!------------------");

                        //发送任务状态字给底层(三段轨迹结束，进入两车对准)
                        task_control.traj_end = 2;

                        //++之后三段轨迹规划结束
                        dynamic_id++;

                        //清空_mpc_lane
                        _mpc_lane.waypoints.clear();

                        //分离出一个单独的线程，处理最后一段两车跟随的逻辑
                        std::thread send_last_traj(std::bind(&BehaviourStateMachine::threadSendLastTraj, this));
                        send_last_traj.detach();

                        continue;
                    }

                    ROS_INFO("---------------send No.%d sub_goal------------------", dynamic_id + 1);
                    pre_sub_goal = sub_goal_vec[dynamic_id]; //记录下新一段轨迹的子目标点

                    bool ret = false;
                    mpc_msgs::Lane temp_lane;
                    //尝试10次规划
                    int retry_cnt = 10;
                    while (retry_cnt--)
                    {
                        ROS_INFO("---------------try %d times planning------------------", 10 - retry_cnt);

                        //发送第一个子目标点并将返回轨迹保存到temp_lane中
                        ret = sendGoalSrv(sub_goal_vec[dynamic_id], temp_lane);

                        //如果规划成功，退出
                        if (ret)
                        {
                            break;
                        }
                    }

                    //如果规划失败了，回到初始状态
                    if (!ret)
                    {
                        ROS_INFO("---------------Hybrid Astar Planning Failed------------------");

                        dynamic_id = 4;

                        //发送任务状态字给底层(直接返回初始状态)
                        task_control.traj_end = 0;

                        continue;
                    }

                    if (use_complex_lane)
                    {
                        checkIsComplexLaneAndPrase(temp_lane, sub_lane_vec);
                    }

                    //如果不是复杂轨迹，直接把整条轨迹发出去
                    if (!is_complex_lane)
                    {
                        _mpc_lane = temp_lane;
                        dynamic_id++;
                    }
                    else
                    {
                        ROS_INFO("have %d complex sub goal", (int)sub_lane_vec.size());
                        dynamic_complex_id = 1;
                        continue;
                    }
                }
                else //没到的话就跳过
                {
                    continue;
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

            continue;
        }
    }
}

void BehaviourStateMachine::waitingCarChangeToParkingGear(bool is_use_gear)
{
    while (true)
    {
        if (is_use_gear)
        {
            if ((_vehicle_status.gear == 1) || (_vehicle_status.gear == 2))
            {
                ROS_INFO("---------------waiting gear changed------------------");
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }
            else
            {
                return;
            }
        }
        else
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

void BehaviourStateMachine::readTrajFile(mpc_msgs::Lane &send_lane, std::string file_path)
{
    std::ifstream fin(file_path, std::ios::in);
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
    while (true)
    {
        //控制帧率
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        //前场重卡跟随
        if (mode != (int)ScenarioStatus::MultiTrajPlanning)
        {
            continue;
        }

        //收到1代表小车和大车位姿细调完成，进入发送跟随轨迹阶段
        while (true)
        {

            ROS_INFO("waiting car pose adjust completed......need task_end==1 , now task_end==%d", (int)task_status.task_end);

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            //如果收到了重置信号，返回初始状态
            if ((int)task_status.task_error == 1 || dynamic_id != 4)
            {
                ROS_WARN("end follow thread");

                //发送任务状态字给底层(直接返回初始状态)
                task_control.traj_end = 0;

                return;
            }

            if ((int)task_status.task_end != 1)
            {
                continue;
            }
            else
            {
                break;
            }
        }

        mpc_msgs::Lane send_lane;
        readTrajFile(send_lane, follow_file_path);
        if (send_lane.waypoints.empty())
        {
            ROS_ERROR("can't open traj_file in threadSendLastTraj()");
            continue;
        }

        ROS_INFO("---------------send last follow lane------------------ , size:%d", (int)send_lane.waypoints.size());

        //发送最后一段固定轨迹
        _mpc_lane = send_lane;

        while (true)
        {

            // std::cout << std::this_thread::get_id() << std::endl;
            ROS_INFO("waiting two car follow lane completed......need task_end==2 , now task_end==%d", (int)task_status.task_end);

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            //如果收到了重置信号，返回初始状态
            if ((int)task_status.task_error == 1 || dynamic_id != 4)
            {
                ROS_WARN("end follow thread");

                //发送任务状态字给底层(直接返回初始状态)
                task_control.traj_end = 0;

                return;
            }

            //收到2代表两车跟随轨迹完成，进入最终的结束阶段并结束该线程
            if ((int)task_status.task_end == 2 || (int)task_status.task_end == 0)
            {
                if ((int)task_status.task_end == 2)
                    ROS_INFO("---------------enter last traj end point---------------");
                else if ((int)task_status.task_end == 0)
                    ROS_INFO("---------------return to initial state---------------");

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

                break;
            }
        }
    }
}

void BehaviourStateMachine::praseTrajFile(const mpc_msgs::Lane &in_lane, std::vector<geometry_msgs::Pose> &stop_points)
{
    //解析后场路径中停止的点，提取出对应的坐标
    for (auto wp : in_lane.waypoints)
    {
        if (wp.direction == 10.0)
        {
            geometry_msgs::Pose point = wp.pose.pose;

            stop_points.push_back(point);

            ROS_INFO("get zero pose:%f,%f", point.position.x, point.position.y);
        }
    }
}

bool BehaviourStateMachine::isStopPointNearby(const geometry_msgs::Pose &stop_pose)
{

    //获取距离差值
    double traj_x = stop_pose.position.x;
    double traj_y = stop_pose.position.y;
    double cur_x = _current_pose_stamped.pose.position.x;
    double cur_y = _current_pose_stamped.pose.position.y;

    auto getDis = [](const double &cur_x, const double &cur_y, const double &traj_x, const double &traj_y)
    {
        double dis = std::hypot(cur_x - traj_x, cur_y - traj_y);
        return dis;
    };
    double dis_error = getDis(cur_x, cur_y, traj_x, traj_y);

    //获取角度差值
    double cur_theta = tf2::getYaw(_current_pose_stamped.pose.orientation) * 180.0 / M_PI;
    double traj_theta = tf2::getYaw(stop_pose.orientation) * 180.0 / M_PI;
    double theta_error = std::fabs(cur_theta - traj_theta);

    if (is_show_debug)
        ROS_INFO("dis_error:%f,theta_error:%f", dis_error, theta_error);

    if (dis_error <= stop_distance && theta_error <= stop_theta)
    {
        return true;
    }
    else
    {
        return false;
    }
    return false;
}

void BehaviourStateMachine::threadPathTracingAndStop()
{
    while (true)
    {
        if (mode != (int)ScenarioStatus::PathTracing)
        {
            continue;
        }

        //从文件中获取固定轨迹
        mpc_msgs::Lane fixed_lane;
        std::string fixed_lane_file_path = fixed_traj_path;
        readTrajFile(fixed_lane, fixed_lane_file_path);

        //解析固定轨迹，得到停止点数组
        std::vector<geometry_msgs::Pose> stop_points;
        praseTrajFile(fixed_lane, stop_points);
        ROS_INFO("get %d stop point", (int)stop_points.size());

        //不停车
        stop_flag = false;
        //发送_mpc_lane
        _mpc_lane = fixed_lane;

        //从当前位置距离路径的最近点开始处理
        int closest_index = -1;
        getClosestIndex(fixed_lane, closest_index);

        last_stop_collision_car_index = closest_index;

        std::size_t index = 0;
        while (true)
        {
            //控制帧率
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            if (index == stop_points.size())
            {
                ROS_INFO("enter last stop position,exit1");
                break;
            }

            //接受到定位信息才继续
            if (!_current_pose_flag)
            {
                ROS_INFO("wait for getting current pose");
                continue;
            }
            //判断当前车体坐标是否到达停止点附近
            bool is_nearby = isStopPointNearby(stop_points[index]);

            if (is_nearby)
            {
                ROS_INFO("-----------------enter No.%d stop pose-----------------", (int)(index + 1));

                //停车
                stop_flag = true;

                //休眠一定时间，等待重卡上称结束
                ROS_INFO("sleep %d second", stop_time);
                std::this_thread::sleep_for(std::chrono::seconds(stop_time));

                if (index == 0)
                {
                    //如果是第一个停止点，之后速度加快
                    waypoints_velocity = prev_waypoints_velocity * max_velo_ratio;
                    start_waypoints_velocity = prev_start_waypoints_velocity * start_velo_ratio;
                }
                //不停车
                stop_flag = false;

                //防止车在只有一个停止点的时候陷入循环（因为index会变成0重来）
                int wait_time = 5;
                ROS_INFO("wait for car driver out of stop position in %d second", wait_time);
                std::this_thread::sleep_for(std::chrono::seconds(5));
                ROS_INFO("wait end");

                //指向下一个停止点
                index++;
            }
            else
            {
                continue;
            }
        }
        if (index == stop_points.size())
        {

            ROS_INFO("enter last stop position,exit2");
            break;
        }
    }
}

void BehaviourStateMachine::run()
{
    signal(SIGINT, MySigintHandler);

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