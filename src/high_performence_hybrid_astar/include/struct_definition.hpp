#ifndef STRUCT_DEFINITION_H
#define STRUCT_DEFINITION_H

// std C++
#include <vector>
#include <memory>

// ros
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>

//搜索状态
enum class SearchStatus
{
    SUCCESS,
    //在起点有碰撞
    FAILURE_COLLISION_AT_START,
    //在终点有碰撞
    FAILURE_COLLISION_AT_GOAL,
    //超时
    FAILURE_TIMEOUT_EXCEEDED,
    //没找到路径
    FAILURE_NO_PATH_FOUND
};

// A星节点状态
enum class NodeStatus
{
    None,
    Open,
    Close,
    Obstacle
};

//车辆形状结构体（单位：米）
struct VehicleShape
{
    double length;
    double width;
    //车辆重心到后轴的距离
    double cg2back;
};

//规划基本参数
struct PlannerCommonParam
{
    //最长搜索时间（ms）
    double time_limit;
    //车体形状结构体
    VehicleShape vehiche_shape;
    //最大转弯半径
    double max_turning_radius;
    //最小转弯半径
    double min_turning_radius;
    //最大和最小转弯半径之间离散的数量
    int turning_radius_size;

    //[0-2π]之间角度离散的数量
    int theta_size;

    //倒车的惩罚权重
    double reverse_weight;
    //改变方向的惩罚权重
    double turning_weight;
    //离终点的横向距离容忍大小
    double goal_lateral_tolerance;
    //离终点的纵向距离容忍大小
    double goal_longitudinal_tolerance;
    //离终点的角度容忍大小
    double goal_angular_tolerance;
    //在costmap下被认为是障碍物的最小值
    int obstacle_threshold;

    //是否允许倒车
    bool use_back;
    //是否使用RS代价值
    bool use_reeds_shepp;
    //是否使用障碍物启发值
    bool use_obstacle_heuristic;
    //是否使用解析扩张
    bool use_analytic_expansion;
    //是否使用航向启发值（小论文）
    bool use_theta_cost;
    //障碍物启发值下的theta_cost（小论文）
    double obstacle_theta_ratio;
    //是否平滑轨迹
    bool use_smoother;

    //与障碍物的最大距离，如果超过这个距离，就没必要优化了
    float obstacle_distance_max;
    /// falloff rate for the voronoi field
    float alpha;
    /// weight for the obstacle term
    float obstacle_weight;
    /// weight for the curvature term
    float curvature_weight;
    /// weight for the smoothness term
    float smoothness_weight;

    //解析扩张的比率
    float analytic_expansion_ratio;
    //解析扩张的最大距离，一般不小于4、5倍的最小转弯半径
    float analytic_expansion_max_length;
};

//单个路点结构体
struct SingleWaypoint
{
    geometry_msgs::PoseStamped pose;
    //是否在倒车状态
    bool is_back = false;
};

//输出轨迹结构体
struct TrajectoryWaypoints
{
    //时间戳和frame信息
    std_msgs::Header header;
    //最终生成的轨迹
    std::vector<SingleWaypoint> trajectory;
};

struct AstarNode
{
    //默认节点状态为None
    NodeStatus status = NodeStatus::None;
    double x;
    double y;
    double theta;
    // g值代价，指路径长度损耗
    double g_cost = 0;
    // h值代价，指启发式函数指
    double h_cost = 0;
    bool is_back = false;
    bool is_turning = false;
    //父亲节点
    std::shared_ptr<AstarNode> parent = nullptr;

    // return f=g+h;
    double getCost() const
    {
        return g_cost + h_cost;
    }
};

struct AstarNode2d
{
    NodeStatus status = NodeStatus::None;
    int x_index;
    int y_index;
    double g_cost = 0.0;
    double h_cost = 0.0;
    bool is_discovered = false;

    std::shared_ptr<AstarNode2d> parent = nullptr;

    double getCost() const
    {
        return g_cost + h_cost;
    }
};

struct NodeComparison
{
    bool operator()(const std::shared_ptr<AstarNode> lhs, const std::shared_ptr<AstarNode> rhs)
    {
        return lhs->getCost() > rhs->getCost();
    }
};

struct NodeComparison2d
{
    bool operator()(const std::shared_ptr<AstarNode2d> lhs, const std::shared_ptr<AstarNode2d> rhs)
    {
        return lhs->getCost() > rhs->getCost();
    }
};

struct NodeUpdate
{
    double shift_x;
    double shift_y;
    double shift_theta;
    //单次步长，即g值
    double step;
    //是否向后
    bool is_back = false;
    //是否转弯
    bool is_turning = false;

    //计算不同转向角下的车体坐标
    NodeUpdate rotated(const double theta) const
    {
        NodeUpdate result = *this;
        //直角坐标系下逆时针的旋转公式（如果theta为0，也就不变）
        result.shift_x = std::cos(theta) * this->shift_x - std::sin(theta) * this->shift_y;
        result.shift_y = std::sin(theta) * this->shift_x + std::cos(theta) * this->shift_y;
        return result;
    }

    //左右反转
    NodeUpdate flipped() const
    {
        NodeUpdate result = *this;
        result.shift_y = -result.shift_y;
        result.shift_theta = -result.shift_theta;
        return result;
    }

    //前后反转
    NodeUpdate reversed() const
    {
        NodeUpdate result = *this;
        result.shift_x = -result.shift_x;
        result.shift_theta = -result.shift_theta;
        result.is_back = !result.is_back;
        return result;
    }
};

struct IndexXYT
{
    int x_index;
    int y_index;
    int theta_index;
    IndexXYT(int x_index, int y_index, int theta_index)
        : x_index(x_index), y_index(y_index), theta_index(theta_index) {}
};

struct IndexXY
{
    int x_index;
    int y_index;
    IndexXY(int x_index, int y_index)
        : x_index(x_index), y_index(y_index) {}
};

#endif