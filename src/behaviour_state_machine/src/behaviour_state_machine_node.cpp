#include "behaviour_state_machine_node.h"

BehaviourStateMachine::BehaviourStateMachine() : _nh(""), _private_nh("~")
{
    _sub_costmap = _nh.subscribe("global_cost_map", 1, &BehaviourStateMachine::callbackCostMap, this);
    _sub_goal_pose = _nh.subscribe("move_base_simple/goal", 1, &BehaviourStateMachine::callbackGoalPose, this);

    _goal_pose_client = _nh.serviceClient<behaviour_state_machine::GoalPose>("goal_pose_srv");
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

void BehaviourStateMachine::run()
{
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();

        //保证每一次接受到新的callbackGoalPose调用才发出srv.call
        if (id != pre_id)
        {
            pre_id = id;

            _goal_pose_srv.request.pose = _goal_pose_stamped.pose;
            _goal_pose_srv.request.header = _goal_pose_stamped.header;

            if (_goal_pose_client.call(_goal_pose_srv))
            {
                bool is_success = _goal_pose_srv.response.is_success;
                if (is_success)
                {
                    ROS_INFO_STREAM_NAMED("behaviour_state_machine:","Hybrid Astar Successful");
                }
                else
                {
                    ROS_INFO("Hybrid Astar Failed");
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