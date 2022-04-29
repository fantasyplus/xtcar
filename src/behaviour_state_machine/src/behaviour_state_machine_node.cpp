#include "behaviour_state_machine_node.h"

BehaviourStateMachine::BehaviourStateMachine() : _nh(""), _private_nh("~")
{
    /*---------------------subscribe---------------------*/
    _sub_costmap = _nh.subscribe("global_cost_map", 1, &BehaviourStateMachine::callbackCostMap, this);
    _sub_goal_pose = _nh.subscribe("move_base_simple/goal", 1, &BehaviourStateMachine::callbackGoalPose, this);
    _sub_current_pose = _nh.subscribe("gnss_pose", 1, &BehaviourStateMachine::callbackCurrentPose, this);

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
        // _tf_buffer->setUsingDedicatedThread(true);
        tf = _tf_buffer->lookupTransform(target, source, ros::Time(0), ros::Duration(1));
    }
    catch (const tf2::LookupException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    return tf;
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

std::vector<geometry_msgs::PoseStamped> BehaviourStateMachine::multipleTargetGenerator()
{
    double dx[4] = {0.0, 0.0, 1.0, -1.0};
    double dy[4] = {1.0, -1.0, 0.0, 0.0};

    std::vector<geometry_msgs::PoseStamped> res_vec;

    for (int i = 0; i < 4; i++)
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
        _target_tf = getTransform(_costmap_frame_id, _goal_pose_stamped.header.frame_id);

        auto pose_in_costmap_frame = transformPose(temp_pose.pose, _target_tf);

        temp_pose.pose = pose_in_costmap_frame;

        res_vec.push_back(temp_pose);
    }

    return res_vec;
}

void BehaviourStateMachine::run()
{
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();

        if (id != pre_id)
        {
            pre_id = id;
            std::vector<geometry_msgs::PoseStamped> res_vec = multipleTargetGenerator();
            for (size_t i = 0; i < res_vec.size(); i++)
            {
                sendGoalSrv(res_vec[i]);
                sleep(1);
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