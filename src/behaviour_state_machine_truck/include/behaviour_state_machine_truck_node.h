#ifndef BEHAVIOUR_STATE_MACHINE_NODE_H
#define BEHAVIOUR_STATE_MACHINE_NODE_H

// std C++
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <signal.h>
#include <thread>
#include <mutex>
#include <fstream>

// ros
#include <ros/ros.h>
#include <std_msgs/Int8.h>
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
#include "behaviour_state_machine_truck/GoalPose.h"
#include "mpc_msgs/Lane.h"
#include "mpc_msgs/Waypoint.h"
#include "mpc_msgs/ControlCommand.h"
#include "mpc_msgs/VehicleStatus.h"
#include "mpc_msgs/TaskControl.h"
#include "mpc_msgs/TaskStatus.h"

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

enum class ScenarioStatus
{
    Stop,
    MultiTrajPlanning,
    PathTracing,
    StaticExec
};

enum class SpecialPoseStatus
{
    Nothing,
    StopPosition,
    ClimbEnd,
    DownslopeStart
};

void MySigintHandler(int sig)
{
    ROS_INFO("shutting down!");
    ros::shutdown();
    exit(0);
}

class BehaviourStateMachine
{
public:
    BehaviourStateMachine();
    void run();

private:
    ros::NodeHandle _nh, _private_nh;

    //接受goal_pose的srv_server和srv
    ros::ServiceClient _goal_pose_client;
    behaviour_state_machine_truck::GoalPose _goal_pose_srv;

    ros::Subscriber _sub_costmap;
    ros::Subscriber _sub_goal_pose;
    ros::Subscriber _sub_current_pose;
    ros::Subscriber _sub_rviz_start_pose;
    ros::Subscriber _sub_vehicle_status;
    ros::Subscriber _sub_scenario_mode;
    ros::Subscriber _sub_task_status;

    //发送mpc路径的发布者
    ros::Publisher _pub_mpc_lane;
    //发送可视化mpc路径的发布者
    ros::Publisher _pub_vis_mpc_lane;
    //发送可视化车辆碰撞框的发布者
    ros::Publisher _pub_vis_car_path;
    //发送和底层通信话题的发布者
    ros::Publisher _pub_task_control;

    ros::Timer _timer_tf;
    ros::Timer _timer_static_exec;

    double loop_rate; // ros参数

private:
    /*---------------------回调函数相关---------------------*/

    //各类pose
    geometry_msgs::PoseStamped _goal_pose_stamped;
    geometry_msgs::PoseStamped _current_pose_stamped;
    geometry_msgs::PoseStamped _rviz_start_pose_stamped;

    bool _current_pose_flag = false;
    bool _goal_pose_flag = false;

    mpc_msgs::VehicleStatus _vehicle_status;

    void callbackGoalPose(const geometry_msgs::PoseStamped &msg);
    void callbackCostMap(const nav_msgs::OccupancyGridConstPtr &msg);
    void callbackCurrentPose(const geometry_msgs::PoseStamped &msg);
    void callbackRvizStartPose(const geometry_msgs::PoseWithCovarianceStamped &msg);
    void callbackVehicleStatus(const mpc_msgs::VehicleStatus &msg);
    void callbackScenarioMode(const std_msgs::Int8 &msg);
    void callbackTaskStatus(const mpc_msgs::TaskStatus &msg);

private:
    /*---------------------发送mpc_lane相关---------------------*/
    // srv返回的mpc路径
    mpc_msgs::Lane _mpc_lane;

    int id = 0;
    int pre_id = 0;
    int dynamic_id = 4; //用于动态地图中实时生成下一段轨迹的判断，每次接受到新的大目标点时赋值为0
    int dynamic_complex_id = 1;

    double prev_waypoints_velocity; //保存最初的速度值
    double prev_start_waypoints_velocity;
    double waypoints_velocity;       // ros参数
    double start_waypoints_velocity; // ros参数
    double max_velo_ratio;
    double start_velo_ratio;

    //发送mpclane的线程
    void threadPublishMpcLane();

    bool is_complex_lane = false;

    void checkIsComplexLaneAndPrase(mpc_msgs::Lane &temp_lane, std::vector<mpc_msgs::Lane> &sub_lane_vec);
    void processMpcLane(mpc_msgs::Lane &cur_lane, int start, int end, bool is_stop, int closest_idx);

    void sendGoalSrv(geometry_msgs::PoseStamped &pose);
    bool sendGoalSrv(geometry_msgs::PoseStamped &pose, mpc_msgs::Lane &lane);

private:
    /*---------------------获取目标点方向和子目标点生成相关---------------------*/
    Direction dir;
    std::vector<geometry_msgs::PoseStamped> sub_goal_vec;

    Direction getDirection(geometry_msgs::PoseStamped &cur, geometry_msgs::PoseStamped &goal);

    std::vector<geometry_msgs::PoseStamped> multipleTargetGenerator(Direction &dir);

    double getDistance(geometry_msgs::PoseStamped &p1, geometry_msgs::PoseStamped &p2);

private:
    /*---------------------障碍物避碰相关---------------------*/
    double vehicle_length;     // ros参数
    double vehicle_width;      // ros参数
    double vehicle_cg2back;    // ros参数
    double lookahead_distance; // ros参数

    // debug 可视化车辆轮廓
    nav_msgs::Path vis_car_path;

    //代价地图
    nav_msgs::OccupancyGridConstPtr _cost_map_ptr;
    bool is_get_costmap = false;

    void getClosestIndex(const mpc_msgs::Lane &temp_lane, int &closest_index);
    void getCollisionPoseVec(int closest_index,
                             const mpc_msgs::Lane &temp_lane,
                             std::vector<std::pair<geometry_msgs::Pose, double>> &base_pose_vec);
    void computeCollisionIndexVec(const geometry_msgs::Pose &base_pose,
                                  std::vector<std::pair<int, int>> &collision_index_vec,
                                  nav_msgs::Path &vis_car_path);
    void checkCollision(bool &is_collision, double &collision_length_to_cur_pose,
                        std::vector<std::pair<geometry_msgs::Pose, double>> &base_pose_vec);

    geometry_msgs::Pose global2local(const nav_msgs::OccupancyGrid &costmap, const geometry_msgs::Pose &pose_global);
    geometry_msgs::Pose local2global(const nav_msgs::OccupancyGrid &costmap, const geometry_msgs::Pose &pose_local);

private:
    void callbackTimerStaticExec(const ros::TimerEvent &e);

    /*---------------------模式切换相关---------------------*/

    /*---------------------通用---------------------*/
    bool use_gear; // ros参数（是否加入档位判断）
    int mode;      // ros参数（模式选择）

    void waitingCarChangeToParkingGear(bool is_use_gear);

    /*---------------------前场相关---------------------*/
    double sub_goal_tolerance_distance; // ros参数
    bool use_complex_lane;              // ros参数
    double first_horizontal_distance;   // ros参数
    double second_vertical_distance;    // ros参数
    double third_nearby_distance;       // ros参数
    std::string follow_file_path;       // ros 参数

    void threadMultiTrajPlanning();

    /*---------------------后场相关---------------------*/
    double stop_distance;        // ros参数
    double stop_theta;           // ros参数
    int stop_time;               // ros参数
    std::string fixed_traj_path; // ros参数
    bool is_show_debug;          // ros参数
    int back_waypoints_num;      // ros参数
    int front_waypoints_num;     // ros参数
    int start_waypoints_num;     // ros参数
    bool stop_flag;
    int last_stop_collision_car_index = 0;

    void threadPathTracingAndStop();

    void praseTrajFile(const mpc_msgs::Lane &in_lane, std::vector<geometry_msgs::Pose> &stop_points);

    bool isStopPointNearby(const geometry_msgs::Pose &stop_pose);

    void reshapeLane(const mpc_msgs::Lane &in_lane, mpc_msgs::Lane &out_lane, int closest_index);

    void visualMpcLane(const mpc_msgs::Lane &send_lane);

private:
    /*---------------------读写路径相关---------------------*/
    std::string save_file_path; // ros 参数

    void readTrajFile(mpc_msgs::Lane &send_lane, std::string file_path);

    void saveTrajFile(const mpc_msgs::Lane save_lane);

    mpc_msgs::TaskStatus task_status;
    void threadSendLastTraj();

private:
    /*---------------------底层通信相关---------------------*/
    //跟底层通信状态切换的消息
    mpc_msgs::TaskControl task_control;
    void threadSendStatusTopic();

private:
    /*---------------------TF相关---------------------*/
    tf::TransformBroadcaster _tf_broadcaster;
    std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

    geometry_msgs::Pose transformPose(const geometry_msgs::Pose &pose,
                                      const geometry_msgs::TransformStamped &transform);

    geometry_msgs::TransformStamped getTransform(const std::string &target,
                                                 const std::string &source);
};

#endif