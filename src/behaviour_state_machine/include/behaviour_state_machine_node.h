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
#include <nav_msgs/Path.h>

//自定义
#include "behaviour_state_machine/GoalPose.h"
#include "mpc_msgs/Lane.h"
#include "mpc_msgs/Waypoint.h"
#include "mpc_msgs/ControlCommand.h"
#include "mpc_msgs/VehicleStatus.h"

enum class Direction
{
    ForwardLeft,
    ForwardRight,
    BackLeft,
    BackRight,
    Forward,
    Back,
    None
};

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
    ros::Subscriber _sub_rviz_start_pose;
    ros::Subscriber _sub_vehicle_status;

    ros::Publisher _pub_rviz_start_pose;

    ros::Timer _timer_tf;

private:
    //各类pose
    geometry_msgs::PoseStamped _goal_pose_stamped;
    geometry_msgs::PoseStamped _current_pose_stamped;
    geometry_msgs::PoseStamped _rviz_start_pose_stamped;
    // costmap的frame_id
    std::string _costmap_frame_id;

    bool _costmap_flag;
    bool _current_pose_flag;
    bool _goal_pose_flag = false;
    bool _publish_tf_flag;
    bool _rviz_start_flag = false;

    int id = 0;
    int pre_id = 0;
    int dynamic_id = 3; //用于动态地图中实时生成下一段轨迹的判断，每次接受到新的大目标点时赋值为0
    int dynamic_complex_id = 1;

    bool is_static_map;   // ros参数
    bool is_keep_sending; // ros参数

    double _sub_goal_tolerance_distance; // ros参数
    mpc_msgs::VehicleStatus _vehicle_status;

private:
    /*---------------------各类回调函数---------------------*/

    void callbackGoalPose(const geometry_msgs::PoseStamped &msg);
    void callbackCostMap(const nav_msgs::OccupancyGrid &msg);
    void callbackCurrentPose(const geometry_msgs::PoseStamped &msg);
    void callbackRvizStartPose(const geometry_msgs::PoseWithCovarianceStamped &msg);
    void callbackVehicleStatus(const mpc_msgs::VehicleStatus &msg);

    void callbackTimerPublishTargetTF(const ros::TimerEvent &e);

private:
    // srv返回的mpc路径
    mpc_msgs::Lane _mpc_lane;

    //发送mpclane的定时器
    ros::Timer _timer_pub_lane;
    //发送mpc路径的发布者
    ros::Publisher _pub_mpc_lane;
    //发送可视化mpc路径的发布者
    ros::Publisher _pub_vis_mpc_lane;

    //定义路径倒数1/_zero_vel_segment的路径点速度为0的参数
    int _zero_vel_segment; // ros参数

    bool is_pub_mpc_lane = false;
    void callbackTimerPublishMpcLane(const ros::TimerEvent &e);

    bool is_complex_lane = false;
    bool use_complex_lane; // ros参数
    void checkIsComplexLaneAndPrase(mpc_msgs::Lane &temp_lane, std::vector<mpc_msgs::Lane> &sub_lane_vec);

    void sendGoalSrv(geometry_msgs::PoseStamped &pose);
    bool sendGoalSrv(geometry_msgs::PoseStamped &pose, mpc_msgs::Lane &lane);

private:
    Direction dir;
    std::vector<geometry_msgs::PoseStamped> sub_goal_vec;
    double first_horizontal_distance; // ros参数
    double second_vertical_distance;  // ros参数
    double third_nearby_distance;     // ros参数

    Direction getDirection(geometry_msgs::PoseStamped &cur, geometry_msgs::PoseStamped &goal);

    std::vector<geometry_msgs::PoseStamped> multipleTargetGenerator(Direction &dir);

    double getDistance(geometry_msgs::PoseStamped &p1, geometry_msgs::PoseStamped &p2);

    void experimentalUse();

private:
    /*---------------------TF相关---------------------*/
    tf::TransformBroadcaster _tf_broadcaster;
    std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

    geometry_msgs::Pose transformPose(const geometry_msgs::Pose &pose,
                                      const geometry_msgs::TransformStamped &transform);

    geometry_msgs::TransformStamped getTransform(const std::string &target,
                                                 const std::string &source);

    void publishTF();
};

#endif