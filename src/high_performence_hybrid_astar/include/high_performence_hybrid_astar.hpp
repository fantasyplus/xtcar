#ifndef HIGH_PERFORMENCE_HYBRID_ASTAR_H
#define HIGH_PERFORMENCE_HYBRID_ASTAR_H

//自定义头文件
#include "struct_definition.hpp"
#include "timer.h"
#include "dynamicvoronoi.h"
#include "vector2d.h"

// std C++
#include <iostream>
#include <vector>
#include <queue>
#include <functional>
#include <string>
#include <tuple>
#include <chrono>
#include <cmath>
#include <unordered_set>
#include <unordered_map>

// ros
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2/utils.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>

// ompl
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>
#include <ompl/base/ScopedState.h>

typedef ompl::base::SE2StateSpace::StateType State;
using AstarNodePtr = std::shared_ptr<AstarNode>;
using AstarNode2dPtr = std::shared_ptr<AstarNode2d>;

class HybridAstar
{
public:
    HybridAstar()
    {
        _pub_open_node = _nh.advertise<visualization_msgs::MarkerArray>("/open_nodes", 1);
        _pub_vis_analytic = _nh.advertise<nav_msgs::Path>("analytic_path", 1);
    };

    // HybridAstar(const PlannerCommonParam &planner_common_param);

    ~HybridAstar() {}

    //设置全局costmap用于规划
    void SetOccupancyGrid(const nav_msgs::OccupancyGrid &cost_map);

    SearchStatus makePlan(const geometry_msgs::Pose &start_pose, const geometry_msgs::Pose &goal_pose);

    //返回最终轨迹
    const TrajectoryWaypoints &getTrajectory() const;

    //清空open_list
    void reset();

    //初始化参数并生成_transition_table
    void initParam(const PlannerCommonParam &planner_common_param);

    //可视化解析扩张路径
    void visualAnalyticPath();

    //清空可视化
    void visualCollisionClear();

    //可视化open node
    void visualOpenNode();

    //轨迹平滑函数
    void smoothPath(TrajectoryWaypoints &initial_traj);

private:
    /*---------------------辅助函数---------------------*/

    // https://arxiv.org/pdf/1708.05551.pdf
    const double kDoublePi = 2.0 * M_PI;
    template <typename T>
    const T wrap_angle(T angle) noexcept
    {
        auto help_angle = angle + T(M_PI);
        while (help_angle < T{})
        {
            help_angle += T(kDoublePi);
        }
        while (help_angle >= T(kDoublePi))
        {
            help_angle -= T(kDoublePi);
        }
        return help_angle - T(M_PI);
    }

    //将角度保持在[min_rad, min_rad + 2π]区间内
    double normalizeRadian(const double rad, const double min_rad = -M_PI);

    //获取当前角度的索引值
    int getThetaIndex(const double theta, const int theta_size);

    //将真实世界坐标变换成costmap索引
    IndexXYT pose2index(const geometry_msgs::Pose &pose_local,
                        const float &costmap_resolution,
                        const int theta_size);

    //将costmap索引变换成真实世界坐标
    geometry_msgs::Pose index2pose(const IndexXYT &index,
                                   const float &costmap_resolution,
                                   const int theta_size);
    //角度转弧度
    constexpr double deg2rad(const double deg);

    //计算欧几里得距离
    double calcDistance2d(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2);
    //计算欧几里得距离
    double calcDistance2d(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2);

    //将yaw角转化为quaternion四元数
    inline geometry_msgs::Quaternion getQuaternion(const double yaw);

    //计算pose在base_pose参考系下的坐标
    geometry_msgs::Pose calcRelativePose(const geometry_msgs::Pose &base_pose, const geometry_msgs::Pose &pose);

    // A*节点转换为ros pose
    geometry_msgs::Pose AstarNode2Pose(const AstarNode &node);

private:
    /*---------------------tf变换相关---------------------*/
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    //获取指定frame之间的变换矩阵
    geometry_msgs::TransformStamped getTransform(const string &from, const string &to);

    //用指定变换矩阵变换位姿
    geometry_msgs::Pose transformPose(const geometry_msgs::Pose &pose,
                                      const geometry_msgs::TransformStamped &transform);

    //从costmap的frame变换到栅格地图起点
    geometry_msgs::Pose global2local(const nav_msgs::OccupancyGrid &costmap,
                                     const geometry_msgs::Pose &pose_global);

    //从栅格地图起点变换到costmap的frame
    geometry_msgs::Pose local2global(const nav_msgs::OccupancyGrid &costmap,
                                     const geometry_msgs::Pose &pose_local);

private:
    /*---------------------碰撞检测相关---------------------*/

    //计算车辆模型的碰撞区间，就是将指定旋转角下的车体映射格点索引保存下来，以备之后用来检测碰撞
    void computeCollisionIndexes(const int theta_index,
                                 std::vector<IndexXY> &index_2d);

    /*类的成员函数后面加 const，表明这个函数不会对这个类对象的数据成员（准确地说是非静态数据成员）作任何改变。*/
    //检测当前格点下是否有碰撞
    bool detectCollision(const IndexXYT &base_index) const;

    //当前索引是否越界
    inline bool isOutOfRange(const IndexXYT &index) const;

    //当前索引是否为障碍物
    inline bool isObstacle(const IndexXYT &index) const;

private:
    /*---------------------核心函数---------------------*/

    //创建节点扩张所需的状态转移表
    std::vector<std::vector<NodeUpdate>> createTransitionTable(const double minimum_turning_radius,
                                                               const double maximum_turning_radius,
                                                               const int turning_radius_size,
                                                               const int theta_size,
                                                               const bool use_back);

    //开始搜索
    SearchStatus search();

    //从搜索终点获取整条路径
    void setPath(AstarNodePtr goal);

    //起点可行性判断
    bool setStartNode();

    //终点可行性判断
    bool setGoalNode();

    //计算启发式值，即h值
    double estimateCost(const geometry_msgs::Pose &start);

    //计算距离启发值
    double getDistanceHeuristic(const geometry_msgs::Pose &start, const geometry_msgs::Pose &goal);

    //计算障碍物启发值
    double getObstacleHeuristic(const geometry_msgs::Pose &start, const geometry_msgs::Pose &goal);

    //是否为终点
    bool isGoal(const AstarNode &node);

    //获取当前索引下对应的A*节点
    AstarNodePtr getNodeRef(const IndexXYT &index);

private:
    /*---------------------轨迹优化函数---------------------*/
    //权重系数
    /// falloff rate for the voronoi field
    float alpha = 0.1;
    /// weight for the obstacle term
    float wObstacle = 0.2;
    /// weight for the curvature term
    float wCurvature = 0.5;
    /// weight for the smoothness term
    float wSmoothness = 0.2;

    Vector2D obstacleTerm(Vector2D xi);

    Vector2D curvatureTerm(Vector2D xim1, Vector2D xi, Vector2D xip1);

    Vector2D smoothnessTerm(Vector2D xim2, Vector2D xim1, Vector2D xi, Vector2D xip1, Vector2D xip2);

private:
    /*---------------------解析扩张函数---------------------*/
    AstarNodePtr tryAnalyticExpansion(AstarNodePtr current_node,
                                      AstarNodePtr goal_node,
                                      int &analytic_iterations,
                                      float &closest_distance);

private:
    //规划参数结构体
    PlannerCommonParam _planner_common_param;

    //用于规划的代价地图
    nav_msgs::OccupancyGrid _cost_map;

    //用于轨迹平滑的维诺图
    DynamicVoronoi _dynamic_voronoi;

    //车体碰撞索引表
    std::vector<std::vector<IndexXY>> _collision_index_table;
    //障碍物表
    std::vector<std::vector<bool>> _is_obstacle_table;

    //存储这次搜索OPEN过的点
    std::unordered_set<AstarNodePtr> _memory_open_nodes;

    /*
    扩张时的状态转移表
    第一维是theta_size个角度，
    第二维是每个角度下，不同转弯半径下的采样点
    */
    std::vector<std::vector<NodeUpdate>> _transition_table;
    // A*节点表（x,y,theta）
    std::vector<std::vector<std::vector<AstarNodePtr>>> _nodes;
    //用于A*搜索时的优先队列
    std::priority_queue<AstarNodePtr, std::vector<AstarNodePtr>, NodeComparison> _open_list;

    //起始位置
    geometry_msgs::Pose _start_pose;
    //目标位置
    geometry_msgs::Pose _goal_pose;
    //最终生成的轨迹
    TrajectoryWaypoints _final_traj;

private:
    /*---------------------2D A*相关(障碍物启发值)---------------------*/
    std::vector<std::vector<AstarNode2dPtr>> _2d_nodes;
    std::priority_queue<AstarNode2dPtr, std::vector<AstarNode2dPtr>, NodeComparison2d> _2d_open_list;
    std::unordered_set<AstarNode2dPtr> _2d_memory_open_nodes;
    int research_cnt = 0;

private:
    /*---------------------可视化相关定义---------------------*/
    ros::NodeHandle _nh;

    //碰撞格点可视化
    ros::Publisher _pub_vis_collision;
    //解析扩张pose可视化
    ros::Publisher _pub_vis_analytic;
    nav_msgs::Path _analytic_path;
    // OPEN节点可视化
    ros::Publisher _pub_open_node;
    visualization_msgs::MarkerArray _open_nodes;

    struct color
    {
        float red;
        float green;
        float blue;
    };

    static constexpr color teal = {102.f / 255.f, 217.f / 255.f, 239.f / 255.f};

    static constexpr color green = {166.f / 255.f, 226.f / 255.f, 46.f / 255.f};

    static constexpr color orange = {253.f / 255.f, 151.f / 255.f, 31.f / 255.f};

    static constexpr color pink = {249.f / 255.f, 38.f / 255.f, 114.f / 255.f};

    static constexpr color purple = {174.f / 255.f, 129.f / 255.f, 255.f / 255.f};
};

#endif