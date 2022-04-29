#ifndef BEHAVIOUR_STATE_MACHINE_NODE_H
#define BEHAVIOUR_STATE_MACHINE_NODE_H

// std C++
#include <iostream>
#include <string>
#include <vector>

// ros
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <tf2/convert.h>
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

    //接受goal_pose的srv_server和srv
    ros::ServiceClient _goal_pose_client;
    behaviour_state_machine::GoalPose _goal_pose_srv;

    ros::Subscriber _sub_costmap;
    ros::Subscriber _sub_goal_pose;
    ros::Subscriber _sub_current_pose;

    ros::Publisher _pub_rviz_start_pose;

    ros::Timer _timer_tf;

private:
    //存储goal_pose
    geometry_msgs::PoseStamped _goal_pose_stamped;
    geometry_msgs::PoseStamped _current_pose_stamped;
    // costmap的frame_id
    std::string _costmap_frame_id;

    bool _costmap_flag;
    bool _current_pose_flag;
    bool _goal_pose_flag;
    bool _publish_tf_flag;

    int id = 0;
    int pre_id = 0;

private:
    /*---------------------各类回调函数---------------------*/

    void callbackGoalPose(const geometry_msgs::PoseStamped &msg);
    void callbackCostMap(const nav_msgs::OccupancyGrid &msg);
    void callbackCurrentPose(const geometry_msgs::PoseStamped &msg);
    void callbackTimerPublishTargetTF(const ros::TimerEvent &e);

private:
    void sendGoalSrv(geometry_msgs::PoseStamped &pose);
    std::vector<geometry_msgs::PoseStamped> multipleTargetGenerator();

private:
    tf::TransformBroadcaster _tf_broadcaster;
    std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
    geometry_msgs::TransformStamped _target_tf;

    geometry_msgs::Pose transformPose(const geometry_msgs::Pose &pose,
                                      const geometry_msgs::TransformStamped &transform);

    geometry_msgs::TransformStamped getTransform(const std::string &target,
                                                 const std::string &source);

    void publishTF();
};

#endif