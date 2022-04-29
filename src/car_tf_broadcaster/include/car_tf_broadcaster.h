#ifndef CAR_TF_BROADCASTER_H
#define CAR_TF_BROADCASTER_H

// std c++
#include <iostream>
#include <string>

// ros
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <tf2/convert.h>
#include <nav_msgs/OccupancyGrid.h>

class CarTF
{
public:
    CarTF();

private:
    ros::NodeHandle _nh;
    ros::NodeHandle _private_nh;

    ros::Subscriber _sub_gnss_pose;
    ros::Subscriber _sub_goal_pose;
    ros::Subscriber _sub_cost_map;
    ros::Subscriber _sub_rviz_start_pose;

    ros::Timer _timer_tf;

private:
    double lidar_trans_x, lidar_trans_y, lidar_trans_z;
    double lidar_rotation_roll, lidar_rotation_pitch, lidar_rotation_yaw;
    std::string map_frame, base_link_frame, lidar_frame;
    std::string pose_topic;

    geometry_msgs::PoseStamped _gnss_pose;
    geometry_msgs::PoseStamped _goal_pose_stamped;
    std::string _costmap_frame_id;
    geometry_msgs::PoseWithCovarianceStamped _rivz_start_pose;

    bool _costmap_flag;
    bool _goal_pose_flag;

    bool use_rviz_start;

private:
    void callbackGnssPose(const geometry_msgs::PoseStampedConstPtr &msg);
    void callbackGoalPose(const geometry_msgs::PoseStamped &msg);
    void callbackCostMap(const nav_msgs::OccupancyGrid &msg);
    void callbackRvizStartPose(const geometry_msgs::PoseWithCovarianceStamped &msg);

private:
    tf::TransformBroadcaster _tf_broadcaster;
    std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
    geometry_msgs::TransformStamped _target_tf;

    void callbackTimerPublishTF(const ros::TimerEvent &e);
    void publishTF();
    geometry_msgs::Pose transformPose(const geometry_msgs::Pose &pose,
                                      const geometry_msgs::TransformStamped &transform);

    geometry_msgs::TransformStamped getTransform(const std::string &target,
                                                 const std::string &source);
};

#endif