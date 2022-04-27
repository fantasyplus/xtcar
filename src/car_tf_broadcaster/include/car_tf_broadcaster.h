#ifndef CAR_TF_BROADCASTER_H
#define CAR_TF_BROADCASTER_H
#include <nmea_msgs/Sentence.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Header.h>
#include <iostream>
#include <ros/ros.h>

class CarTF
{
public:
    CarTF();

private:
    ros::NodeHandle _nh;
    ros::NodeHandle _private_nh;

    tf::TransformBroadcaster _tf_broadcaster;

    double lidar_trans_x, lidar_trans_y, lidar_trans_z;
    double lidar_rotation_roll, lidar_rotation_pitch, lidar_rotation_yaw;
    std::string map_frame, base_link_frame, lidar_frame;
    std::string pose_topic;
    double dt;

    geometry_msgs::PoseStamped _gnss_pose;
    ros::Subscriber sub_gnss_pose;
    ros::Timer timer_tf;

private:
    void callbackTimerPublishTF(const ros::TimerEvent &e);
    void callbackGnssPose(const geometry_msgs::PoseStampedConstPtr &msg);
    void publishTF();
};

#endif