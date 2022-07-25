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
#include "behaviour_state_machine/GoalPose.h"
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
    behaviour_state_machine::GoalPose _goal_pose_srv;

    ros::Subscriber _sub_costmap;
    ros::Subscriber _sub_goal_pose;
    ros::Subscriber _sub_current_pose;
    ros::Subscriber _sub_rviz_start_pose;
    ros::Subscriber _sub_vehicle_status;
    ros::Subscriber _sub_scenario_mode;
    ros::Subscriber _sub_task_status;

    ros::Timer _timer_tf;

    double loop_rate;

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

    //发送mpc路径的发布者
    ros::Publisher _pub_mpc_lane;
    //发送可视化mpc路径的发布者
    ros::Publisher _pub_vis_mpc_lane;

    int id = 0;
    int pre_id = 0;
    int dynamic_id = 4; //用于动态地图中实时生成下一段轨迹的判断，每次接受到新的大目标点时赋值为0
    int dynamic_complex_id = 1;

    double waypoints_velocity; // ros参数

    //发送mpclane的线程
    void threadPublishMpcLane();

    bool is_complex_lane = false;

    void checkIsComplexLaneAndPrase(mpc_msgs::Lane &temp_lane, std::vector<mpc_msgs::Lane> &sub_lane_vec);
    void processMpcLane(mpc_msgs::Lane &cur_lane, int start, int end, bool is_stop);

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
    double vehicle_length;  // ros参数
    double vehicle_width;   // ros参数
    double vehicle_cg2back; // ros参数

    double lookahead_distance; // ros参数

    // debug 可视化车辆轮廓
    nav_msgs::Path vis_car_path;
    ros::Publisher _pub_vis_car_path;

    //代价地图
    nav_msgs::OccupancyGridConstPtr _cost_map_ptr;
    bool is_get_costmap = false;

    //保存最后一次在前探距离上发生碰撞时车体在路径上的位置索引
    static int last_collision_car_index;

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
    ros::Timer _timer_static_exec;
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

    void threadMultiTrajPlanning();

    /*---------------------后场相关---------------------*/
    double stop_distance; // ros参数
    double stop_theta;    // ros参数

    void threadPathTracingAndStop();

    //解析后场路径中速度为0的点，提取出对应的坐标
    void praseTrajFile(const mpc_msgs::Lane &in_lane, std::vector<geometry_msgs::Pose> &stop_points);

    bool isStopPointNearby(const geometry_msgs::Pose &stop_pose);

private:
    /*---------------------读写路径相关---------------------*/
    std::string follow_file_path; // ros 参数
    std::string save_file_path;   // ros 参数

    void readTrajFile(mpc_msgs::Lane &send_lane, std::string file_path);

    void saveTrajFile(const mpc_msgs::Lane save_lane);

    mpc_msgs::TaskStatus task_status;
    void threadSendLastTraj();

private:
    /*---------------------底层通信相关---------------------*/
    //跟底层通信状态切换的消息
    mpc_msgs::TaskControl task_control;
    ros::Publisher _pub_task_control;
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