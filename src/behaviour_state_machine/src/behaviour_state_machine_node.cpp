#include "behaviour_state_machine_node.h"

BehaviourStateMachine::BehaviourStateMachine() : _nh(""), _private_nh("~")
{
    _private_nh.param<bool>("is_static_map", is_static_map, true);

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

    /*---------------------advertise---------------------*/
    _pub_rviz_start_pose = _nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, true);

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
    _rviz_start_pose_stamped.header = msg.header;
    _rviz_start_pose_stamped.pose = msg.pose.pose;
}

void BehaviourStateMachine::callbackGoalPose(const geometry_msgs::PoseStamped &msg)
{
    id++;
    _goal_pose_stamped = msg;
}

void BehaviourStateMachine::callbackCostMap(const nav_msgs::OccupancyGrid &msg)
{
    _costmap_frame_id = msg.header.frame_id;
}

void BehaviourStateMachine::callbackCurrentPose(const geometry_msgs::PoseStamped &msg)
{
    _current_pose_stamped = msg;
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
        std::vector<double> right_dy{2, 0, 0};

        dx.insert(dx.begin(), right_dx.begin(), right_dx.end());
        dy.insert(dy.begin(), right_dy.begin(), right_dy.end());

        break;
    }
    default:
        break;
    }

    std::vector<geometry_msgs::PoseStamped> res_vec;

    for (std::size_t i = 0; i < dx.size(); i++)
    {
        geometry_msgs::PoseStamped temp_pose;

        temp_pose.header.seq = _goal_pose_stamped.header.seq;
        temp_pose.header.stamp = _goal_pose_stamped.header.stamp;
        temp_pose.header.frame_id = _costmap_frame_id; //最终是要转换成costmap下的坐标
        temp_pose.pose = _goal_pose_stamped.pose;

        double theta = tf2::getYaw(_goal_pose_stamped.pose.orientation);
        temp_pose.pose.position.x += dx[i] * std::cos(theta) - dy[i] * std::sin(theta);
        temp_pose.pose.position.y += dx[i] * std::sin(theta) + dy[i] * std::cos(theta);

        // ROS_INFO("_costmap_frame_id:%s , _goal_pose_stamped:%s", _costmap_frame_id.c_str(), _goal_pose_stamped.header.frame_id.c_str());
        auto _target_tf = getTransform(_costmap_frame_id, _goal_pose_stamped.header.frame_id);

        auto pose_in_costmap_frame = transformPose(temp_pose.pose, _target_tf);

        temp_pose.pose = pose_in_costmap_frame;

        res_vec.push_back(temp_pose);
    }

    return res_vec;
}

void BehaviourStateMachine::experimentalUse()
{
    geometry_msgs::PoseWithCovarianceStamped temp_start;
    temp_start.header.stamp = ros::Time::now();
    temp_start.header.frame_id = "map";
    temp_start.pose.pose.position.x = 9.9479637146;
    temp_start.pose.pose.position.y = 12.3065662384;
    temp_start.pose.pose.position.z = 0.0;
    temp_start.pose.pose.orientation.x = 0.0;
    temp_start.pose.pose.orientation.y = 0.0;
    temp_start.pose.pose.orientation.z = 0.766014177738;
    temp_start.pose.pose.orientation.w = 0.642823676839;
    _pub_rviz_start_pose.publish(temp_start);

    _goal_pose_stamped.header.stamp = ros::Time::now();
    _goal_pose_stamped.header.frame_id = "map";
    _goal_pose_stamped.pose.position.x = 39.7019958496;
    _goal_pose_stamped.pose.position.y = 38.8793182373;
    _goal_pose_stamped.pose.position.z = 0.0;
    _goal_pose_stamped.pose.orientation.x = 0.0;
    _goal_pose_stamped.pose.orientation.y = 0.0;
    _goal_pose_stamped.pose.orientation.z = 0.688019505944;
    _goal_pose_stamped.pose.orientation.w = 0.725692193317;

    // sleep(1);
    sendGoalSrv(_goal_pose_stamped);
}

void BehaviourStateMachine::run()
{
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();

        //保证每次接受到新的目标点才发送srv
        if (id != pre_id)
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
                // Direction dir = getDirection(_rviz_start_pose_stamped, _goal_pose_stamped);
                // std::vector<geometry_msgs::PoseStamped> res_vec = multipleTargetGenerator(dir);
                // geometry_msgs::PoseWithCovarianceStamped next_start;

                // for (std::size_t i = 0; i < res_vec.size(); i++)
                // {
                //     sendGoalSrv(res_vec[i]);

                //     //把这次的目标定为下次的起点
                //     next_start.header = res_vec[i].header;
                //     next_start.pose.pose = res_vec[i].pose;
                //     sleep(1);
                //     _pub_rviz_start_pose.publish(next_start);
                // }
            }
            else
            {
                Direction dir = getDirection(_current_pose_stamped, _goal_pose_stamped);
                std::vector<geometry_msgs::PoseStamped> res_vec = multipleTargetGenerator(dir);
                for (std::size_t i = 0; i < res_vec.size(); i++)
                {
                    sendGoalSrv(res_vec[i]);
                    sleep(1);
                }
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