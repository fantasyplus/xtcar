#ifndef BEHAVIOUR_STATE_MACHINE_NODE_H
#define BEHAVIOUR_STATE_MACHINE_NODE_H

// std C++
#include <iostream>

// ros
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

//自定义
#include "behaviour_state_machine/GoalPose.h"

class BehaviourStateMachine
{
public:
    BehaviourStateMachine();

private:
    ros::NodeHandle _nh, _private_nh;
};

#endif