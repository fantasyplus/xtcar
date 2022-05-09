#include <ros/ros.h>
// #include <mpc_msgs/Lane.h>
// #include <mpc_msgs/Waypoint.h>
#include <mpc_msgs/Waypoint.h>
#include "turtlesim/Pose.h"
//#include "chasis_driver/VehicleStatus.h"
#include "std_msgs/String.h"
// #include "mpc_follower/qp_solver/qp_solver_unconstr_fast.h"
// #include "mpc_follower/mpc_trajectory.h"
// #include <mpc_follower/amathutils_lib/amathutils.hpp>
//#include "mpc_follower/mpc_utils.h"
#include "mpc_follower/mpc_follower_core.h"
#include <qpOASES.hpp>

int main(int argc, char **argv)
{
    // ROS节点初始化
    
    ros::init(argc, argv, "test_node");

    qpOASES::SQProblem solver_;
    amathutils::normalizeRadian(5);
    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Publisher，发布名为/info的topic
    ros::Publisher person_info_pub = n.advertise<std_msgs::String>("/info", 10);

    // 设置循环的频率
    ros::Rate loop_rate(1);

    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;
        msg.data = "Hello World!";
        // 发布消息
		person_info_pub.publish(msg);

       	ROS_INFO("Hello World!");
        //ROS_INFO_STREAM("Hello World!");
       // 按照循环频率延时
        loop_rate.sleep();
    }

    return 0;
}