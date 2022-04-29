#ifndef BEHAVIOUR_STATE_MACHINE_NODE_H
#define BEHAVIOUR_STATE_MACHINE_NODE_H

// std C++
#include <iostream>
#include <string>

// ros
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/OccupancyGrid.h>

//自定义
#include "behaviour_state_machine/GoalPose.h"

class BehaviourStateMachine
{
public:
    BehaviourStateMachine();
    void run();

private:
    ros::NodeHandle _nh, _private_nh;

    //接受goal_pose的srv客户端
    ros::ServiceClient _goal_pose_client;
    //接受goal_pose的srv
    behaviour_state_machine::GoalPose _goal_pose_srv;

    ros::Subscriber _sub_costmap;
    ros::Subscriber _sub_goal_pose;

private:
    //存储goal_pose
    geometry_msgs::PoseStamped _goal_pose_stamped;

    // costmap的frame_id
    std::string _costmap_frame_id;
    
    int id=0;
    int pre_id=0;

private:
    /*---------------------各类回调函数---------------------*/

    void callbackGoalPose(const geometry_msgs::PoseStamped &msg);
    void callbackCostMap(const nav_msgs::OccupancyGrid &msg);
};

#endif