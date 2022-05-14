#include "behaviour_state_machine_node.h"

BehaviourStateMachine::BehaviourStateMachine() : _nh(""), _private_nh("~")
{
    _private_nh.param<bool>("is_static_map", is_static_map, true);
    _private_nh.param<bool>("is_keep_sending", is_keep_sending, false);
    _private_nh.param<double>("sub_goal_tolerance_distance", _sub_goal_tolerance_distance, 1.0);
    _private_nh.param<int>("zero_vel_segment", _zero_vel_segment, 1);
    _private_nh.param<bool>("use_complex_lane", use_complex_lane, false);

    /*---------------------subscribe---------------------*/
    _sub_costmap = _nh.subscribe("global_cost_map", 1, &BehaviourStateMachine::callbackCostMap, this);
    _sub_goal_pose = _nh.subscribe("move_base_simple/goal", 1, &BehaviourStateMachine::callbackGoalPose, this);
    if (is_static_map)
    {
        _sub_rviz_start_pose = _nh.subscribe("initialpose", 1, &BehaviourStateMachine::callbackRvizStartPose, this);
    }
    else
    {
        _sub_current_pose = _nh.subscribe("gnss_pose", 1, &BehaviourStateMachine::callbackCurrentPose, this);
    }

    _sub_vehicle_status = _nh.subscribe("vehicle_status", 1, &BehaviourStateMachine::callbackVehicleStatus, this);

    /*---------------------advertise---------------------*/
    _pub_rviz_start_pose = _nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, true);
    _pub_mpc_lane = _nh.advertise<mpc_msgs::Lane>("mpc_waypoints", 1, true);
    _pub_vis_mpc_lane = _nh.advertise<nav_msgs::Path>("vis_mpc_waypoints", 1, true);

    //发布mpclane的定时器
    _timer_pub_lane = _nh.createTimer(ros::Duration(0.2), &BehaviourStateMachine::callbackTimerPublishMpcLane, this);

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

void BehaviourStateMachine::callbackRvizStartPose(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
    _rviz_start_flag = true;

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
    if (!is_static_map && _current_pose_flag) // 在动态地图下才会进入三段轨迹的生成
    {
        dir = getDirection(_current_pose_stamped, _goal_pose_stamped);
        sub_goal_vec = multipleTargetGenerator(dir);
    }
}

void BehaviourStateMachine::callbackCostMap(const nav_msgs::OccupancyGrid &msg)
{
    _costmap_frame_id = msg.header.frame_id;
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

void BehaviourStateMachine::callbackTimerPublishMpcLane(const ros::TimerEvent &e)
{
    if (is_pub_mpc_lane)
    {
        /*-----对mpclane进行处理,并发送真实路径-----*/

        //到这里的_mpc_lane应该都是单段轨迹，即只前进或只后退
        //对后1/_zero_vel_segment的路径点速度赋0
        int size = _mpc_lane.waypoints.size();
        // ROS_INFO("_mpc_lane.size:%d", (int)size);

        if (size >= _zero_vel_segment)
        {
            //获得开始赋0的起始点位置
            int zero_point_start_index = size - (int)(((double)size / (double)_zero_vel_segment) + 0.5); //四舍五入
            // ROS_INFO("(int)std::round(size / _zero_vel_segment:%d", (int)(((double)size / (double)_zero_vel_segment) + 0.5));
            for (int i = zero_point_start_index - 1; i < size; i++)
            {
                _mpc_lane.waypoints[i].twist.twist.linear.x = 0.0;
            }
        }
        else
        { //如果size比_zero_vel_segment还小，给最后一个点速度赋0
            _mpc_lane.waypoints[size - 1].twist.twist.linear.x = 0.0;
        }

        //最后一个点的direction赋6（停止标志位）
        _mpc_lane.waypoints[size - 1].direction = 6;
        _pub_mpc_lane.publish(_mpc_lane);

        /*-----发送可视化路径-----*/
        nav_msgs::Path vis_lane;
        vis_lane.header.frame_id = "map";
        vis_lane.header.stamp = ros::Time::now();

        // mpc里的点本来就是在map下的
        for (std::size_t i = 0; i < _mpc_lane.waypoints.size(); i++)
        {
            geometry_msgs::PoseStamped temp_pose;
            temp_pose = _mpc_lane.waypoints[i].pose;

            vis_lane.poses.push_back(temp_pose);
        }

        _pub_vis_mpc_lane.publish(vis_lane);
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

void BehaviourStateMachine::sendGoalSrv(geometry_msgs::PoseStamped &pose, mpc_msgs::Lane &mpc_lane)
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
        }
        else
        {
            ROS_INFO("Hybrid Astar Failed");
            mpc_lane = temp_lane;
        }
    }
    mpc_lane = temp_lane;
}

Direction BehaviourStateMachine::getDirection(geometry_msgs::PoseStamped &cur, geometry_msgs::PoseStamped &goal)
{
    geometry_msgs::TransformStamped goal2cur_tf;
    goal2cur_tf = getTransform("base_link", goal.header.frame_id);

    geometry_msgs::Pose pose_in_cur_frame = transformPose(goal.pose, goal2cur_tf);

    // ROS_INFO("pose_in_cur_frame:%f,%f,frame:%s", pose_in_cur_frame.position.x, pose_in_cur_frame.position.y, goal2cur_tf.header.frame_id.c_str());
    if (pose_in_cur_frame.position.y > 0)
    {
        ROS_INFO("Target at Current's Left");
        return Direction::Left;
    }
    else if (pose_in_cur_frame.position.y < 0)
    {
        ROS_INFO("Target at Current's Right");
        return Direction::Right;
    }
    else if (pose_in_cur_frame.position.y == 0)
    {
        return Direction::Straight;
    }

    return Direction::None;
}

std::vector<geometry_msgs::PoseStamped> BehaviourStateMachine::multipleTargetGenerator(Direction &dir)
{
    std::vector<double> dx;
    std::vector<double> dy;
    switch (dir)
    {
    //右前前
    case Direction::Left:
    {
        std::vector<double> left_dx{0, 4, 2};
        std::vector<double> left_dy{-2, 0, 0};

        dx.insert(dx.begin(), left_dx.begin(), left_dx.end());
        dy.insert(dy.begin(), left_dy.begin(), left_dy.end());

        break;
    }
    //左前前
    case Direction::Right:
    {
        std::vector<double> right_dx{0, 4, 2};
        std::vector<double> right_dy{-2, 0, 0};

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

void BehaviourStateMachine::experimentalUse()
{
    geometry_msgs::PoseWithCovarianceStamped temp_start;
    temp_start.header.stamp = ros::Time::now();
    temp_start.header.frame_id = "map";
    temp_start.pose.pose.position.x = 44.5119094849;
    temp_start.pose.pose.position.y = 28.0472793579;
    temp_start.pose.pose.position.z = 0.0;
    temp_start.pose.pose.orientation.x = 0.0;
    temp_start.pose.pose.orientation.y = 0.0;
    temp_start.pose.pose.orientation.z = -0.368646407278;
    temp_start.pose.pose.orientation.w = 0.929569699593;
    _pub_rviz_start_pose.publish(temp_start);

    _goal_pose_stamped.header.stamp = ros::Time::now();
    _goal_pose_stamped.header.frame_id = "map";
    _goal_pose_stamped.pose.position.x = 80.6284713745;
    _goal_pose_stamped.pose.position.y = 41.0094299316;
    _goal_pose_stamped.pose.position.z = 0.0;
    _goal_pose_stamped.pose.orientation.x = 0.0;
    _goal_pose_stamped.pose.orientation.y = 0.0;
    _goal_pose_stamped.pose.orientation.z = -0.0824805305618;
    _goal_pose_stamped.pose.orientation.w = 0.996592676111;

    sendGoalSrv(_goal_pose_stamped);
}

void BehaviourStateMachine::run()
{
    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        ros::spinOnce();

        if (!is_static_map) //动态地图下的前置判断
        {
            if (!_current_pose_flag || !_goal_pose_flag)
            {
                continue;
            }
        }

        //每次接受到新的目标点才发送srv
        if (id != pre_id && !is_keep_sending)
        {
            pre_id = id;

            if (is_static_map)
            {
                //实验用
                int cnt = 1;
                while (cnt--)
                {
                    experimentalUse();
                }
            }
        }
        else if (is_keep_sending) //不停的判断，如果满足条件就发送
        {
            if (is_static_map)
            {
                ROS_INFO("-----must set is_static_map to false-----");
                continue;
            }

            static geometry_msgs::PoseStamped pre_sub_goal;
            static std::vector<mpc_msgs::Lane> sub_lane_vec;

            if (dynamic_id != 3) //发送3段轨迹（子目标点）且车辆执行完毕后就停止，直到接受到新的大目标点
            {

                //如果是复杂轨迹(有前进有后退)，先不考虑正常的子目标点生成，对它单独处理
                //不处理完这段轨迹是不会进入后面的流程的
                if (is_complex_lane && use_complex_lane)
                {
                    //如果复杂轨迹数组为空了，说明处理完该段轨迹了，进入正常的流程
                    if (sub_lane_vec.empty())
                    {
                        is_complex_lane = false;

                        dynamic_id++; //转移到下一个正常的子目标点

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
                    //进入发送轨迹流程，mpclane发送标志位赋1
                    is_pub_mpc_lane = true;

                    int sub_turn_index = sub_lane_vec[0].waypoints.size() - 1;
                    geometry_msgs::PoseStamped pre_complex_sub_goal; //当前子轨迹的终点
                    pre_complex_sub_goal = sub_lane_vec[0].waypoints[sub_turn_index].pose;

                    double length = getDistance(_current_pose_stamped, pre_complex_sub_goal);
                    // ROS_INFO("remain length:%f", length);

                    if (length < _sub_goal_tolerance_distance && _vehicle_status.speed < 1e-5) //判断车辆是否到达前一段轨迹的终点
                    {
                        ROS_INFO("arrive at no.%d pre_complex_sub_goal", (int)sub_lane_vec.size());
                        //删除当前轨迹
                        sub_lane_vec.erase(sub_lane_vec.begin());

                        //直接进入下一次循环，避免在复杂轨迹处理完之前进入到正常流程
                        continue;
                    }
                }

                //正常简单轨迹流程
                if (dynamic_id == 0 && !is_complex_lane) //第一段
                {
                    ROS_INFO("---------------send first sub_goal------------------");

                    pre_sub_goal = sub_goal_vec[dynamic_id]; //记录下第一段的子目标点

                    mpc_msgs::Lane temp_lane;
                    sendGoalSrv(sub_goal_vec[dynamic_id], temp_lane); //发送第一个子目标点并将返回轨迹保存到temp_lane中

                    if (use_complex_lane)
                    {
                        checkIsComplexLaneAndPrase(temp_lane, sub_lane_vec);
                    }

                    //如果不是复杂轨迹，直接把整条轨迹发出去
                    if (!is_complex_lane)
                    {
                        _mpc_lane = temp_lane;
                        //进入发送轨迹流程，mpclane发送标志位赋1
                        is_pub_mpc_lane = true;
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

                    if (length < _sub_goal_tolerance_distance && _vehicle_status.speed < 1e-5) //判断车辆是否到达前一段轨迹的终点
                    {
                        ROS_INFO("---------------send next sub_goal------------------");
                        pre_sub_goal = sub_goal_vec[dynamic_id]; //记录下新一段轨迹的子目标点

                        mpc_msgs::Lane temp_lane;
                        sendGoalSrv(sub_goal_vec[dynamic_id], temp_lane); //发送子目标点并将返回轨迹保存到temp_lane中

                        if (use_complex_lane)
                        {
                            checkIsComplexLaneAndPrase(temp_lane, sub_lane_vec);
                        }

                        //如果不是复杂轨迹，直接把整条轨迹发出去
                        if (!is_complex_lane)
                        {
                            _mpc_lane = temp_lane;
                            //进入发送轨迹流程，mpclane发送标志位赋1
                            is_pub_mpc_lane = true;
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
                        // ROS_INFO("haven't get sub_goal");
                        continue;
                    }
                }
            }
            else
            {
                //重置静态变量，为下一次接受大目标点做准备
                geometry_msgs::PoseStamped empty_pose;
                pre_sub_goal = empty_pose;

                // mpclane发送标志位赋为0
                is_pub_mpc_lane = false;

                //重置复杂轨迹数组
                sub_lane_vec.clear();
                continue;
            }
        }

        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "behaviour_state_machine");

    BehaviourStateMachine obj;
    obj.run();

    return 0;
}