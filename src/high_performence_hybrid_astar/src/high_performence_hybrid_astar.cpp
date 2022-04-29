#include "high_performence_hybrid_astar.hpp"

// HybridAstar::HybridAstar(const PlannerCommonParam &planner_common_param)
//     : _planner_common_param(planner_common_param)
// {
//     _transition_table = createTransitionTable(
//         _planner_common_param.min_turning_radius,
//         _planner_common_param.max_turning_radius,
//         _planner_common_param.turning_radius_size,
//         _planner_common_param.theta_size,
//         _planner_common_param.use_back);
// }

void HybridAstar::initParam(const PlannerCommonParam &planner_common_param)
{
    _planner_common_param = planner_common_param;
    _transition_table = createTransitionTable(
        _planner_common_param.min_turning_radius,
        _planner_common_param.max_turning_radius,
        _planner_common_param.turning_radius_size,
        _planner_common_param.theta_size,
        _planner_common_param.use_back);

    alpha = _planner_common_param.alpha;
    wObstacle = _planner_common_param.obstacle_weight;
    wCurvature = _planner_common_param.curvature_weight;
    wSmoothness = _planner_common_param.smoothness_weight;
}

std::vector<std::vector<NodeUpdate>> HybridAstar::createTransitionTable(const double minimum_turning_radius,
                                                                        const double maximum_turning_radius,
                                                                        const int turning_radius_size,
                                                                        const int theta_size,
                                                                        const bool use_back)
{
    //遍历每一个角度
    std::vector<std::vector<NodeUpdate>> transition_table;
    transition_table.resize(theta_size);

    const double dtheta = 2.0 * M_PI / static_cast<double>(theta_size);

    // arc  = r * theta

    const auto &R_min = minimum_turning_radius;
    const auto &R_max = maximum_turning_radius;

    //每一步更新的最小移动距离，赋给走直线的情况
    //  const double step_min = R_min * dtheta;
    double step_min = std::sqrt(2) * _cost_map.info.resolution; //单次步长一定会走出一个格子
    step_min = (R_min * dtheta) < step_min ? R_min * dtheta : step_min;

    const double dR = (R_max - R_min) / static_cast<double>(turning_radius_size);

    std::vector<NodeUpdate> forward_node_candidates;

    //走直线的情况
    const NodeUpdate forward_straight{step_min, 0.0, 0.0, step_min, false, false};
    forward_node_candidates.push_back(forward_straight);

    //计算从最小转弯半径到最大转弯半径下的位置偏移（单位角度下）
    for (int i = 0; i < static_cast<int>(turning_radius_size + 1); ++i)
    {
        double R = R_min + i * dR; // R从最小增加到最大

        // double step = R * dtheta;  //单次路径长度为曲线的弧长（也就是gcost）
        //和最短步长比较
        double step = (R * dtheta) > step_min ? R * dtheta : step_min;

        NodeUpdate forward_left{R * sin(dtheta), R * (1 - cos(dtheta)), dtheta, step, false, true};
        NodeUpdate forward_right = forward_left.flipped();
        forward_node_candidates.push_back(forward_left);
        forward_node_candidates.push_back(forward_right);
    }

    //遍历整个角度空间，dtheta为单位角度
    for (int i = 0; i < theta_size; i++)
    {
        const double theta = dtheta * static_cast<double>(i);

        for (const auto &nu : forward_node_candidates)
        {
            //插入在当前角度下的候选点（从最小转弯半径到最大转弯半径）
            transition_table[i].push_back(nu.rotated(theta));
        }

        if (use_back)
        {
            for (const auto &nu : forward_node_candidates)
            {
                transition_table[i].push_back(nu.reversed().rotated(theta));
            }
        }
    }

    return transition_table;
}

SearchStatus HybridAstar::makePlan(const geometry_msgs::Pose &start_pose, const geometry_msgs::Pose &goal_pose)
{

    _start_pose = global2local(_cost_map, start_pose);
    _goal_pose = global2local(_cost_map, goal_pose);

    ROS_INFO("get start pose:%f,%f", _start_pose.position.x, _start_pose.position.y);
    ROS_INFO("get goal pose:%f,%f", _goal_pose.position.x, _goal_pose.position.y);

    if (!setStartNode())
    {
        // ROS_INFO("start_pose is:%f,%f", start_pose.position.x, start_pose.position.y);
        return SearchStatus::FAILURE_COLLISION_AT_START;
    }

    if (!setGoalNode())
    {
        return SearchStatus::FAILURE_COLLISION_AT_GOAL;
    }

    return search();
}

SearchStatus HybridAstar::search()
{
    TimerClock t0;

    ROS_INFO("----------start search----------");

    //碰撞节点可视化id
    int vis_collision_coount = 0;

    while (!_open_list.empty())
    {
        //当前时间减去初始时间
        const double msec = t0.getTimerMilliSec();
        //超时
        if (msec > _planner_common_param.time_limit)
        {
            return SearchStatus::FAILURE_TIMEOUT_EXCEEDED;
        }

        //选择代价值最小的节点作为扩张节点
        AstarNodePtr current_node = _open_list.top();

        //也放入记忆set，用于最后的reset（以防万一,估计就是起点和终点）
        _memory_open_nodes.insert(current_node);

        //判断是否已经为Close（比如倒车的情况）
        if (current_node->status == NodeStatus::Close)
        {
            // ROS_INFO("current_node has closed");
            _open_list.pop();
            continue;
        }

        _open_list.pop();
        current_node->status = NodeStatus::Close;

        //尝试解析扩张
        AstarNodePtr goal_node = std::make_shared<AstarNode>();
        goal_node->x = _goal_pose.position.x;
        goal_node->y = _goal_pose.position.y;
        goal_node->theta = tf2::getYaw(_goal_pose.orientation);

        int analytic_iterations = 0;
        float closest_distance = std::numeric_limits<float>::max();

        AstarNodePtr analytic_node = tryAnalyticExpansion(current_node, goal_node, analytic_iterations, closest_distance);
        if (analytic_node != nullptr)
        {
            ROS_INFO("analytic expansion successfully");
            current_node = analytic_node;
            goal_node.reset();
        }

        //如果当前节点就是终点，那么记录路径并返回SUCCESS
        if (isGoal(*current_node))
        {
            setPath(current_node);
            return SearchStatus::SUCCESS;
        }

        //开始扩张
        const auto index_theta = getThetaIndex(current_node->theta, _planner_common_param.theta_size);
        for (const auto &transition : _transition_table[index_theta])
        {
            const bool is_back_point = transition.is_back != current_node->is_back;
            const bool is_turning_point = transition.is_turning != current_node->is_turning;

            // g值，即真实路径代价值
            double move_cost = 0.0;
            double back_cost = 0.0;
            //如果是反向的话，需要乘上反向惩罚权重
            back_cost = is_back_point ? _planner_common_param.reverse_weight * transition.step : transition.step;
            //如果有转弯，需要乘上转弯惩罚权重
            move_cost = is_turning_point ? _planner_common_param.turning_weight * back_cost : back_cost;

            //计算下一个状态的索引
            geometry_msgs::Pose next_pose;
            next_pose.position.x = current_node->x + transition.shift_x;
            next_pose.position.y = current_node->y + transition.shift_y;
            next_pose.orientation = getQuaternion(current_node->theta + transition.shift_theta);
            const auto next_index = pose2index(next_pose, _cost_map.info.resolution, _planner_common_param.theta_size);

            //如果扩张的这个点有碰撞，跳过该点
            if (detectCollision(next_index))
            {
                // visualization_msgs::Marker collision_cube;
                // vis_collision_coount++;

                // collision_cube.header.frame_id = "map";
                // collision_cube.header.stamp = ros::Time::now();
                // collision_cube.type = visualization_msgs::Marker::CUBE;

                // collision_cube.id = vis_collision_coount;

                // collision_cube.scale.x = _cost_map.info.resolution;
                // collision_cube.scale.y = _cost_map.info.resolution;
                // collision_cube.scale.z = 0.1;

                // collision_cube.color.a = 0.5;
                // collision_cube.color.r = orange.red;
                // collision_cube.color.g = orange.green;
                // collision_cube.color.b = orange.blue;

                // collision_cube.pose.position.x = next_index.x_index;
                // collision_cube.pose.position.y = next_index.y_index;

                // _collision_cubes.markers.push_back(collision_cube);
                continue;
            }

            /*
            1.如果下一个点状态为None，直接更新它
            2.如果下一个点状态为Open，说明它之前被扩张过，
            但是这一次，当前点的g值+当前点到下一个点的g值（next_gc），
            小于它之前被扩张过记录的g值（next_node->gc），
            那么就更新它
            */
            AstarNodePtr next_node = getNodeRef(next_index);

            const double next_gc = current_node->g_cost + move_cost;
            if (next_node->status == NodeStatus::None || next_gc < next_node->g_cost)
            {
                next_node->status = NodeStatus::Open;
                next_node->x = next_pose.position.x;
                next_node->y = next_pose.position.y;
                next_node->theta = tf2::getYaw(next_pose.orientation);
                next_node->g_cost = next_gc;
                next_node->h_cost = estimateCost(next_pose);
                next_node->is_back = transition.is_back;
                next_node->is_turning = transition.is_turning;
                next_node->parent = current_node;
                _open_list.push(next_node);
                _memory_open_nodes.insert(next_node);

                continue;
            }
        }
    }

    //没找到路径
    return SearchStatus::FAILURE_NO_PATH_FOUND;
}

bool HybridAstar::setStartNode()
{

    const auto start_index = pose2index(_start_pose, _cost_map.info.resolution, _planner_common_param.theta_size);

    //检测初始节点是否有碰撞
    if (detectCollision(start_index))
    {
        return false;
    }

    //设置A*初始节点
    AstarNodePtr start_node = getNodeRef(start_index);
    start_node->x = _start_pose.position.x;
    start_node->y = _start_pose.position.y;
    start_node->theta = 2.0 * M_PI / static_cast<double>(_planner_common_param.theta_size) *
                        static_cast<double>(start_index.theta_index);
    start_node->g_cost = 0;
    start_node->h_cost = estimateCost(_start_pose);
    start_node->is_back = false;
    start_node->status = NodeStatus::Open;
    start_node->parent = nullptr;

    //将初始节点推入openlist
    _open_list.push(start_node);

    return true;
}

bool HybridAstar::setGoalNode()
{
    const auto goal_index = pose2index(_goal_pose, _cost_map.info.resolution, _planner_common_param.theta_size);

    /*
    检测终点出是否有碰撞
    这种检测方法只适合全局地图的情况
    */
    if (detectCollision(goal_index))
    {
        return false;
    }

    return true;
}

void HybridAstar::setPath(AstarNodePtr goal_node)
{
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = _cost_map.header.frame_id;

    _final_traj.header = header;
    _final_traj.trajectory.clear();

    //从目标点反向迭代回起点
    AstarNodePtr node = goal_node;

    while (node != nullptr)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = header;
        pose.pose = local2global(_cost_map, AstarNode2Pose(*node));

        SingleWaypoint single_waypoint;
        single_waypoint.pose = pose;
        single_waypoint.is_back = node->is_back;
        _final_traj.trajectory.push_back(single_waypoint);

        //迭代至下一个点
        node = node->parent;
    }

    //反转路径，才是从起点到终点的路径
    std::reverse(_final_traj.trajectory.begin(), _final_traj.trajectory.end());

    //更新第一个点的朝向
    if (_final_traj.trajectory.size() > 1)
    {
        _final_traj.trajectory.at(0).is_back = _final_traj.trajectory.at(1).is_back;
    }
}

double HybridAstar::getDistanceHeuristic(const geometry_msgs::Pose &start, const geometry_msgs::Pose &goal)
{
    ompl::base::ReedsSheppStateSpace reedsSheppPath(_planner_common_param.min_turning_radius);

    State *rsStart = (State *)reedsSheppPath.allocState();
    State *rsEnd = (State *)reedsSheppPath.allocState();

    rsStart->setXY(start.position.x, start.position.y);
    rsStart->setYaw(tf2::getYaw(start.orientation));

    rsEnd->setXY(goal.position.x, goal.position.y);
    rsEnd->setYaw(tf2::getYaw(goal.orientation));

    return reedsSheppPath.distance(rsStart, rsEnd);
}

double HybridAstar::getObstacleHeuristic(const geometry_msgs::Pose &start, const geometry_msgs::Pose &goal)
{
}

double HybridAstar::estimateCost(const geometry_msgs::Pose &start)
{
    double total_cost = 0.0;
    if (_planner_common_param.use_reeds_shepp)
    {
        total_cost += getDistanceHeuristic(start, _goal_pose);
    }
    else
    {
        total_cost += calcDistance2d(start, _goal_pose);
    }
    return total_cost;
}

void HybridAstar::SetOccupancyGrid(const nav_msgs::OccupancyGrid &cost_map)
{
    _cost_map = cost_map;

    // ROS_INFO("costmap height:%d,width:%d,resolution:%f", _cost_map.info.height, _cost_map.info.width, _cost_map.info.resolution);

    const auto map_height = _cost_map.info.height;
    const auto map_width = _cost_map.info.width;

    //读取costmap中每个格点的障碍物信息，生成障碍物表
    _is_obstacle_table.clear();
    _is_obstacle_table.resize(map_height);
    for (int i = 0; i < map_height; i++)
    {
        _is_obstacle_table[i].resize(map_width);
        for (int j = 0; j < map_width; j++)
        {
            const auto cost = _cost_map.data[i * map_width + j];

            if (cost < 0 || cost >= _planner_common_param.obstacle_threshold)
            {
                _is_obstacle_table[i][j] = true;
            }
        }
    }

    if (_planner_common_param.use_smoother)
    {
        //生成维诺图，用于之后的轨迹平滑
        bool **bin_map = new bool *[map_height];
        for (int x = 0; x < map_width; x++)
        {
            bin_map[x] = new bool[map_height];
            for (int y = 0; y < map_height; y++)
            {
                bin_map[x][y] = _cost_map.data[y * map_width + x] ? true : false;
            }
        }
        _dynamic_voronoi.initializeMap(map_width, map_height, bin_map);
        _dynamic_voronoi.update();
    }

    //为每一个搜索角度设置一个车体碰撞索引数组
    _collision_index_table.clear();
    for (int theta_index = 0; theta_index < _planner_common_param.theta_size; theta_index++)
    {
        std::vector<IndexXY> index_2d;
        computeCollisionIndexes(theta_index, index_2d);
        _collision_index_table.push_back(index_2d);
    }

    //初始化A*节点表
    _nodes.clear();
    _nodes.resize(map_height);
    for (int i = 0; i < map_height; i++)
    {
        _nodes[i].resize(map_width);
        for (int j = 0; j < map_width; j++)
        {
            _nodes[i][j].resize(_planner_common_param.theta_size);
            for (int t = 0; t < _planner_common_param.theta_size; t++)
            {
                _nodes[i][j][t] = std::make_shared<AstarNode>();
            }
        }
    }

    // ROS_INFO("SetOccupancyGrid successfully");
}

//计算车辆模型的碰撞区间，就是将指定旋转角下的车体映射格点索引保存下来，以备之后用来检测碰撞
void HybridAstar::computeCollisionIndexes(const int theta_index, std::vector<IndexXY> &index_2d)
{
    const auto vehicle_shape = _planner_common_param.vehiche_shape;

    //将车体定义为一个矩形，以重心为原点，右手坐标系，front为x轴正方向，left为y轴正方向
    const double back = -1.0 * vehicle_shape.cg2back;
    const double front = vehicle_shape.length - vehicle_shape.cg2back;
    const double right = -1.0 * vehicle_shape.width / 2;
    const double left = vehicle_shape.width / 2;

    //得到坐标系原点（0，0），和指定旋转角theta
    const auto base_pose = index2pose({0, 0, theta_index}, _cost_map.info.resolution, _planner_common_param.theta_size);
    const double base_theta = tf2::getYaw(base_pose.orientation);

    const float _cost_mapresolution = static_cast<double>(_cost_map.info.resolution);

    //用一个一维数组保存车体大小范围内的格点索引
    for (double x = back; x <= front; x += _cost_mapresolution)
    {
        for (double y = right; y <= left; y += _cost_mapresolution)
        {
            //直角坐标系下逆时针的旋转公式
            //计算不同转向角下的车体坐标（如果base_theta为0，也就不变）
            const double offset_x = x * std::cos(base_theta) - y * std::sin(base_theta);
            const double offset_y = x * std::sin(base_theta) + y * std::cos(base_theta);

            geometry_msgs::Pose traverse_car_pose;
            traverse_car_pose.position.x = base_pose.position.x + offset_x;
            traverse_car_pose.position.y = base_pose.position.y + offset_y;

            const IndexXYT traverse_car_index_xyt = pose2index(traverse_car_pose, _cost_map.info.resolution, _planner_common_param.theta_size);
            IndexXY index_xy(traverse_car_index_xyt.x_index, traverse_car_index_xyt.y_index);
            index_2d.push_back(index_xy);
        }
    }
}

bool HybridAstar::detectCollision(const IndexXYT &base_index) const
{
    //获取当前搜索角度下的车辆碰撞索引数组
    const auto &coll_indexes_2d = _collision_index_table[base_index.theta_index];
    for (const auto &coll_index_2d : coll_indexes_2d)
    {
        // theta对碰撞检测没有影响，所以无所谓取多少
        IndexXYT coll_index{coll_index_2d.x_index, coll_index_2d.y_index, 0};

        //当前点的位置，加上车体大小映射点
        coll_index.x_index += base_index.x_index;
        coll_index.y_index += base_index.y_index;

        if (isOutOfRange(coll_index) || isObstacle(coll_index))
        {
            return true;
        }
    }
    return false;
}

void HybridAstar::visualCollisionClear()
{
    visualization_msgs::MarkerArray clear_cubes;
    visualization_msgs::Marker clear_cube;

    clear_cube.header.frame_id = _cost_map.header.frame_id;
    clear_cube.header.stamp = ros::Time::now();
    clear_cube.id = 0;
    clear_cube.action = 3;
    clear_cubes.markers.push_back(clear_cube);
    _pub_vis_collision.publish(clear_cubes);
}

void HybridAstar::visualAnalyticPath()
{
    // _pub_vis_collision.publish(_collision_cubes);

    _analytic_path.header.frame_id = _cost_map.header.frame_id;
    _analytic_path.header.stamp = ros::Time::now();
    _pub_vis_analytic.publish(_analytic_path);
    _analytic_path.poses.clear();
}

void HybridAstar::visualOpenNode()
{
    int clear_index = 0;
    static int id = 0;
    for (auto it : _memory_open_nodes)
    {
        if (it->status == NodeStatus::Open)
        {
            visualization_msgs::Marker vehicle_marker;

            if (clear_index == 0)
            {
                vehicle_marker.action = 3;
                clear_index = 1;
            }

            vehicle_marker.header.frame_id = _cost_map.header.frame_id;
            vehicle_marker.header.stamp = ros::Time::now();
            vehicle_marker.id = id++;
            vehicle_marker.type = visualization_msgs::Marker::ARROW;

            vehicle_marker.scale.x = _cost_map.info.resolution;
            vehicle_marker.scale.y = _cost_map.info.resolution;
            vehicle_marker.scale.z = 0.1;

            vehicle_marker.color.a = 0.1;
            vehicle_marker.color.r = orange.red;
            vehicle_marker.color.g = orange.green;
            vehicle_marker.color.b = orange.blue;

            geometry_msgs::PoseStamped pose;
            pose.header = vehicle_marker.header;
            pose.pose = local2global(_cost_map, AstarNode2Pose(*it));

            vehicle_marker.pose = pose.pose;

            _open_nodes.markers.push_back(vehicle_marker);
        }
    }
    _pub_open_node.publish(_open_nodes);
}

inline bool HybridAstar::isOutOfRange(const IndexXYT &index) const
{
    if (index.x_index < 0 || index.x_index >= _cost_map.info.width)
    {
        return true;
    }
    if (index.y_index < 0 || index.y_index >= _cost_map.info.height)
    {
        return true;
    }
    return false;
}

inline bool HybridAstar::isObstacle(const IndexXYT &index) const
{
    //在调用这个函数之前，已经进行过边界检查了，所以不用担心越界
    return _is_obstacle_table[index.y_index][index.x_index];
}

double HybridAstar::normalizeRadian(const double rad, const double min_rad)
{
    double radian = wrap_angle(rad);
    return radian < min_rad ? radian += 2.0 * M_PI : radian;
}

int HybridAstar::getThetaIndex(const double theta, const int theta_size)
{
    //当前角度除以单位角，再对离散角的总个数取余，就可以获得当前角在2π下的索引
    const double angle_increment_rad = 2.0 * M_PI / static_cast<double>(theta_size);
    return static_cast<int>(normalizeRadian(theta, 0.0) / angle_increment_rad) %
           static_cast<int>(theta_size);
}

geometry_msgs::TransformStamped HybridAstar::getTransform(const string &from, const string &to)
{
    geometry_msgs::TransformStamped tf;
    try
    {
        // tf_buffer_->setUsingDedicatedThread(true);
        tf = tf_buffer_->lookupTransform(from, to, ros::Time(0), ros::Duration(1));
    }
    catch (const tf2::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    return tf;
}

geometry_msgs::Pose HybridAstar::transformPose(const geometry_msgs::Pose &pose, const geometry_msgs::TransformStamped &transform)
{
    geometry_msgs::PoseStamped transformed_pose;

    geometry_msgs::PoseStamped pose_orig;
    pose_orig.pose = pose;

    // res=transform*pose_orig
    tf2::doTransform(pose_orig, transformed_pose, transform);

    return transformed_pose.pose;
}

geometry_msgs::Pose HybridAstar::global2local(const nav_msgs::OccupancyGrid &costmap, const geometry_msgs::Pose &pose_global)
{
    // costmap的origin，本质就是costmap到目标frame的变换矩阵
    tf2::Transform tf_origin;
    tf2::convert(costmap.info.origin, tf_origin);

    geometry_msgs::TransformStamped transform;
    transform.transform = tf2::toMsg(tf_origin.inverse());

    return transformPose(pose_global, transform);
}

geometry_msgs::Pose HybridAstar::local2global(const nav_msgs::OccupancyGrid &costmap, const geometry_msgs::Pose &pose_local)
{
    // costmap的origin，本质就是costmap到目标frame的变换矩阵
    tf2::Transform tf_origin;
    tf2::convert(costmap.info.origin, tf_origin);

    geometry_msgs::TransformStamped transform;
    transform.transform = tf2::toMsg(tf_origin);

    return transformPose(pose_local, transform);
}

IndexXYT HybridAstar::pose2index(const geometry_msgs::Pose &pose_local, const float &_cost_mapresolution, const int theta_size)
{
    const auto resolution = static_cast<double>(_cost_mapresolution);
    const int index_x = static_cast<int>(std::floor(pose_local.position.x / resolution));
    const int index_y = static_cast<int>(std::floor(pose_local.position.y / resolution));
    const int index_theta = getThetaIndex(tf2::getYaw(pose_local.orientation), theta_size);
    return {index_x, index_y, index_theta};
}

geometry_msgs::Pose HybridAstar::index2pose(const IndexXYT &index, const float &_cost_mapresolution, const int theta_size)
{
    geometry_msgs::Pose pose_local;

    //如果当前位置为（1，1），分辨率为1，那么index等于1。如果分辨率为0.2，那么index等于5。分辨率越小，index越大。
    pose_local.position.x = static_cast<float>(index.x_index + 0.5) * _cost_mapresolution;
    pose_local.position.y = static_cast<float>(index.y_index + 0.5) * _cost_mapresolution;

    const double angle_increment_rad = 2.0 * M_PI / static_cast<double>(theta_size);
    const double yaw = index.theta_index * angle_increment_rad;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw);
    tf2::convert(quat, pose_local.orientation);

    return pose_local;
}

const TrajectoryWaypoints &HybridAstar::getTrajectory() const
{
    return _final_traj;
}

constexpr double HybridAstar::deg2rad(const double deg)
{
    return deg * M_PI / 180.0;
}

double HybridAstar::calcDistance2d(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
    return std::hypot(p2.x - p1.x, p2.y - p1.y);
}

double HybridAstar::calcDistance2d(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2)
{
    return calcDistance2d(p1.position, p2.position);
}

inline geometry_msgs::Quaternion HybridAstar::getQuaternion(const double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    return tf2::toMsg(q);
}

//计算pose在base_pose参考系下的坐标
geometry_msgs::Pose HybridAstar::calcRelativePose(const geometry_msgs::Pose &base_pose, const geometry_msgs::Pose &pose)
{
    tf2::Transform tf_transform;
    tf2::convert(base_pose, tf_transform);

    geometry_msgs::TransformStamped transform;
    transform.transform = tf2::toMsg(tf_transform.inverse());

    return transformPose(pose, transform);
}

geometry_msgs::Pose HybridAstar::AstarNode2Pose(const AstarNode &node)
{
    geometry_msgs::Pose pose;

    pose.position.x = node.x;
    pose.position.y = node.y;
    pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, node.theta);
    pose.orientation = tf2::toMsg(q);

    return pose;
}

bool HybridAstar::isGoal(const AstarNode &node)
{
    const auto relative_pose = calcRelativePose(_goal_pose, AstarNode2Pose(node));

    //大于横向和纵向容忍值
    if (std::fabs(relative_pose.position.x) > _planner_common_param.goal_longitudinal_tolerance ||
        std::fabs(relative_pose.position.y) > _planner_common_param.goal_lateral_tolerance)
    {
        return false;
    }

    //大于角度容忍值
    const auto angle_diff = normalizeRadian(tf2::getYaw(relative_pose.orientation));
    if (std::abs(angle_diff) > _planner_common_param.goal_angular_tolerance)
    {
        return false;
    }

    return true;
}

AstarNodePtr HybridAstar::getNodeRef(const IndexXYT &index)
{
    return _nodes[index.y_index][index.x_index]
                 [index.theta_index];
}

void HybridAstar::reset()
{
    std::priority_queue<AstarNodePtr, std::vector<AstarNodePtr>, NodeComparison> empty_list;
    std::swap(_open_list, empty_list);

    //初始化A*节点表
    //遍历所有点
    // for (int i = 0; i < _cost_map.info.height; i++)
    // {
    //     for (int j = 0; j < _cost_map.info.width; j++)
    //     {
    //         for (int t = 0; t < _planner_common_param.theta_size; t++)
    //         {
    //             if (_nodes[i][j][t]->status != NodeStatus::None)
    //             {
    //                 _nodes[i][j][t]->g_cost = 0.0;
    //                 _nodes[i][j][t]->status = NodeStatus::None;
    //                 _nodes[i][j][t]->h_cost = 0.0;
    //             }
    //         }
    //     }
    // }

    //遍历这次OPEN或者CLOSE过的点
    for (auto it : _memory_open_nodes)
    {
        it->g_cost = 0.0;
        it->status = NodeStatus::None;
        it->h_cost = 0.0;
    }
    std::unordered_set<AstarNodePtr> empty_set;
    std::swap(_memory_open_nodes, empty_set);
}

//判断前后共5个点是否有方向不一致的问题，即判断一段距离方向是否改变
inline bool isCusp(std::vector<SingleWaypoint> &traj, int index)
{
    bool dir_before_2 = traj[index - 2].is_back ? true : false;
    bool dir_before_1 = traj[index - 1].is_back ? true : false;
    bool dir_current = traj[index].is_back ? true : false;
    bool dir_after_1 = traj[index + 1].is_back ? true : false;
    bool dir_after_2 = traj[index + 2].is_back ? true : false;

    if (dir_before_2 != dir_before_1 || dir_before_1 != dir_current || dir_current != dir_after_1 || dir_after_1 != dir_after_2)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void HybridAstar::smoothPath(TrajectoryWaypoints &initial_traj)
{

    const int voronoi_width = _dynamic_voronoi.getSizeX();
    const int voronoi_height = _dynamic_voronoi.getSizeY();

    //最大迭代次数
    const int max_interations = 500;
    int iteration = 0;

    int path_length = initial_traj.trajectory.size();

    std::vector<SingleWaypoint> smooth_traj = initial_traj.trajectory;

    float total_weight = wSmoothness + wCurvature + wObstacle; //四项的权重数
    while (iteration < max_interations)
    {
        for (int i = 2; i < path_length - 2; i++)
        {
            if (isCusp(smooth_traj, i))
            {
                continue;
            }

            Vector2D wp_before_2(smooth_traj[i - 2].pose.pose.position.x,
                                 smooth_traj[i - 2].pose.pose.position.y);

            Vector2D wp_before_1(smooth_traj[i - 1].pose.pose.position.x,
                                 smooth_traj[i - 1].pose.pose.position.y);

            Vector2D wp_current(smooth_traj[i].pose.pose.position.x,
                                smooth_traj[i].pose.pose.position.y);

            Vector2D wp_after_1(smooth_traj[i + 1].pose.pose.position.x,
                                smooth_traj[i + 1].pose.pose.position.y);

            Vector2D wp_after_2(smooth_traj[i + 2].pose.pose.position.x,
                                smooth_traj[i + 2].pose.pose.position.y);

            Vector2D correction;
            geometry_msgs::Pose temp_correction;

            /*----------障碍物项----------*/
            correction = correction - obstacleTerm(wp_current);

            //边界判断
            {
                temp_correction.position.x = correction.getX();
                temp_correction.position.y = correction.getY();
                const IndexXYT index_correction = pose2index(temp_correction,
                                                             _cost_map.info.resolution,
                                                             _planner_common_param.theta_size);

                if (isOutOfRange(index_correction))
                {
                    continue;
                }
            }

            /*----------平滑项----------*/
            correction = correction - smoothnessTerm(wp_before_2,
                                                     wp_before_1,
                                                     wp_current,
                                                     wp_after_1,
                                                     wp_after_2);

            //边界判断
            {
                temp_correction.position.x = correction.getX();
                temp_correction.position.y = correction.getY();
                const IndexXYT index_correction = pose2index(temp_correction,
                                                             _cost_map.info.resolution,
                                                             _planner_common_param.theta_size);
                if (isOutOfRange(index_correction))
                {
                    continue;
                }
            }

            /*----------曲率限制项----------*/
            correction = correction - curvatureTerm(wp_before_1,
                                                    wp_current,
                                                    wp_after_1);
            //边界判断
            {
                temp_correction.position.x = correction.getX();
                temp_correction.position.y = correction.getY();
                const IndexXYT index_correction = pose2index(temp_correction,
                                                             _cost_map.info.resolution,
                                                             _planner_common_param.theta_size);
                if (isOutOfRange(index_correction))
                {
                    continue;
                }
            }

            wp_current = wp_current + alpha * correction / total_weight;
            smooth_traj[i].pose.pose.position.x = wp_current.getX();
            smooth_traj[i].pose.pose.position.y = wp_current.getY();

            Vector2D d_theta = wp_current - wp_before_1;
            smooth_traj[i - 1].pose.pose.orientation = getQuaternion(std::atan2(d_theta.getY(), d_theta.getX()));
        }
        iteration++;
    }

    initial_traj.trajectory = smooth_traj;
}

/*!
   \fn float clamp(float n, float lower, float upper)
   \brief Clamps a number between a lower and an upper bound
   \param t heading in rad
*/
inline float clamp(float n, float lower, float upper)
{
    return std::max(lower, std::min(n, upper));
}

Vector2D HybridAstar::obstacleTerm(Vector2D xi)
{
    Vector2D gradient;
    // the distance to the closest obstacle from the current node
    float obsDst = _dynamic_voronoi.getDistance(xi.getX(), xi.getY());
    // the vector determining where the obstacle is
    int x = (int)xi.getX();
    int y = (int)xi.getY();
    // if the node is within the map
    if (x < _cost_map.info.width && x >= 0 && y < _cost_map.info.height && y >= 0)
    {
        //从当前点xi到最近障碍点的向量
        Vector2D obsVct(xi.getX() - _dynamic_voronoi.data[(int)xi.getX()][(int)xi.getY()].obstX,
                        xi.getY() - _dynamic_voronoi.data[(int)xi.getX()][(int)xi.getY()].obstY);

        // the closest obstacle is closer than desired correct the path for that
        if (obsDst < _planner_common_param.obstacle_distance_max)
        {
            //参考：
            // Dolgov D, Thrun S, Montemerlo M, et al. Practical search techniques in path planning for
            //  autonomous driving[J]. Ann Arbor, 2008, 1001(48105): 18-80.
            return gradient = wObstacle * 2 * (obsDst - _planner_common_param.obstacle_distance_max) * obsVct / obsDst;
        }
    }
    return gradient;
}

Vector2D HybridAstar::curvatureTerm(Vector2D xim1, Vector2D xi, Vector2D xip1)
{
    Vector2D gradient;
    // the vectors between the nodes
    Vector2D Dxi = xi - xim1;
    Vector2D Dxip1 = xip1 - xi;
    // orthogonal complements vector
    Vector2D p1, p2;

    // the distance of the vectors
    float absDxi = Dxi.length();
    float absDxip1 = Dxip1.length();

    float kappaMax = 1.f / (_planner_common_param.min_turning_radius * 1.1);

    // ensure that the absolute values are not null
    if (absDxi > 0 && absDxip1 > 0)
    {
        // the angular change at the node
        float Dphi = std::acos(clamp(Dxi.dot(Dxip1) / (absDxi * absDxip1), -1, 1));
        float kappa = Dphi / absDxi;

        // if the curvature is smaller then the maximum do nothing
        if (kappa <= kappaMax)
        {
            Vector2D zeros;
            return zeros;
        }
        else
        {
            //代入原文公式(2)与(3)之间的公式
            //参考：
            // Dolgov D, Thrun S, Montemerlo M, et al. Practical search techniques in path planning for
            //  autonomous driving[J]. Ann Arbor, 2008, 1001(48105): 18-80.
            float absDxi1Inv = 1 / absDxi;
            float PDphi_PcosDphi = -1 / std::sqrt(1 - std::pow(std::cos(Dphi), 2));
            float u = -absDxi1Inv * PDphi_PcosDphi;
            // calculate the p1 and p2 terms
            p1 = xi.ort(-xip1) / (absDxi * absDxip1); //公式(4)
            p2 = -xip1.ort(xi) / (absDxi * absDxip1);
            // calculate the last terms
            float s = Dphi / (absDxi * absDxi);
            Vector2D ones(1, 1);
            Vector2D ki = u * (-p1 - p2) - (s * ones);
            Vector2D kim1 = u * p2 - (s * ones);
            Vector2D kip1 = u * p1;

            // calculate the gradient
            gradient = wCurvature * (0.25 * kim1 + 0.5 * ki + 0.25 * kip1);

            if (std::isnan(gradient.getX()) || std::isnan(gradient.getY()))
            {
                std::cout << "nan values in curvature term" << std::endl;
                Vector2D zeros;
                return zeros;
            }
            // return gradient of 0
            else
            {
                return gradient;
            }
        }
    }
    // return gradient of 0
    else
    {
        // std::cout << "abs values not larger than 0" << std::endl;
        Vector2D zeros;
        return zeros;
    }
}

Vector2D HybridAstar::smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2)
{
    return wSmoothness * (xim2 - 4 * xim1 + 6 * xi - 4 * xip1 + xip2);
}

AstarNodePtr HybridAstar::tryAnalyticExpansion(AstarNodePtr current_node,
                                               AstarNodePtr goal_node,
                                               int &analytic_iterations /* 0 */,
                                               float &closest_distance)
{
    //获取当前点到终点的距离，来判断是否需要更频繁的解析扩张
    ompl::base::ReedsSheppStateSpace rs_state(_planner_common_param.min_turning_radius);
    State *rs_start = (State *)rs_state.allocState();
    State *rs_end = (State *)rs_state.allocState();
    rs_start->setXY(current_node->x, current_node->y);
    rs_start->setYaw(current_node->theta);
    rs_end->setXY(goal_node->x, goal_node->y);
    rs_end->setYaw(goal_node->theta);

    float rs_distance = rs_state.distance(rs_start, rs_end);
    closest_distance = std::min(closest_distance, rs_distance);

    //距离越近搜索频率越大
    int desired_iterations = std::max(static_cast<int>(closest_distance / _planner_common_param.analytic_expansion_ratio),
                                      static_cast<int>(std::ceil(_planner_common_param.analytic_expansion_ratio)));

    analytic_iterations = std::min(analytic_iterations, desired_iterations);

    /*analytic_iterations初值为0，所以在刚开始搜索时必会进入if，
    万一有从开头就可以直接生成的路径，可以节省大量时间*/
    if (analytic_iterations <= 0)
    {
        analytic_iterations = desired_iterations;

        /*如果距离太长了，就退出。
        这是为了防止生成不安全的路线，万一进入了cost很高的区域，
        不利于以后的搜索，所以要设立一个安全距离*/
        if (rs_distance > _planner_common_param.analytic_expansion_max_length)
        {
            return nullptr;
        }

        //根号2倍的地图分辨率，保证每一次都会到一个新的格子
        const float min_step = std::sqrt(_cost_map.info.resolution);
        const unsigned int num_intervals = std::ceil(rs_distance / min_step);

        //生成除了终点的曲线中间点
        std::vector<AstarNodePtr> analytic_candidate_nodes;
        std::vector<double> reals;

        static ompl::base::StateSpacePtr state = std::make_shared<ompl::base::ReedsSheppStateSpace>(_planner_common_param.min_turning_radius);
        ompl::base::ScopedState<> from(state), to(state), s(state);
        from[0] = current_node->x;
        from[1] = current_node->y;
        from[2] = current_node->theta;
        to[0] = goal_node->x;
        to[1] = goal_node->y;
        to[2] = goal_node->theta;

        int vis_collision_coount = 0;
        bool is_back;

        //除了起点的插值
        rs_state.clearStaticIndexPrev();
        for (unsigned int i = 1; i <= num_intervals; i++)
        {
            //开始插值
            rs_state.interpolateByXt(from(), to(), (double)i / num_intervals, s(), is_back);
            reals = s.reals();

            //检测该点是否有碰撞
            geometry_msgs::Pose pose_2_collision;
            pose_2_collision.position.x = reals[0];
            pose_2_collision.position.y = reals[1];
            pose_2_collision.orientation = getQuaternion(normalizeRadian(reals[2], 0.0));

            IndexXYT index_2_collision = pose2index(pose_2_collision,
                                                    _cost_map.info.resolution,
                                                    _planner_common_param.theta_size);

            //如果有碰撞的话，直接返回空指针。因为本次解析扩张已经失败了
            if (detectCollision(index_2_collision))
            {
                //发生碰撞的时候，程序并不会结束，但是解析扩张会重来，
                //所以如果不重置prev_i的话，下次解析扩张开始时prev_i还是之前的值并不一定为0
                rs_state.clearStaticIndexPrev();
                // ROS_INFO("-------collision-------");
                return nullptr;
            }
            else
            {
                AstarNodePtr candidate_node = std::make_shared<AstarNode>();
                candidate_node->x = reals[0];
                candidate_node->y = reals[1];
                candidate_node->theta = normalizeRadian(reals[2], 0.0);
                candidate_node->is_back = is_back;

                analytic_candidate_nodes.push_back(candidate_node);
            }
        }

        //建立曲线点的父子关系
        AstarNodePtr prev = current_node;
        for (auto node : analytic_candidate_nodes)
        {
            // ROS_INFO("node back status is: %d", node->is_back);
            node->parent = prev;

            //可视化路径
            geometry_msgs::PoseStamped vis_pose;
            vis_pose.header.frame_id = _cost_map.header.frame_id;
            vis_pose.header.stamp = ros::Time::now();
            vis_pose.pose.position.x = node->x;
            vis_pose.pose.position.y = node->y;
            vis_pose.pose.orientation = getQuaternion(node->theta);
            vis_pose.pose = local2global(_cost_map, vis_pose.pose);
            _analytic_path.poses.push_back(vis_pose);

            prev = node;
        }
        // ROS_INFO("successful analytic node is %d", (int)analytic_candidate_nodes.size());
        return prev;
    }

    //一直减减，直到下一次等于0的时候，再次进入if，就相当于1/analytic_iterations的频率了
    analytic_iterations--;

    return nullptr;
}