#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <mpc_msgs/Lane.h>
#include <mpc_msgs/Waypoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <mpc_msgs/VehicleStatus.h>
#include <fstream>


class MPCSaveTraj{
    
public:
    MPCSaveTraj(){
    
    // ROS_INFO_STREAM("initial inside !");
    front_pose.pose.position.x=-100;
    front_pose.pose.position.y=-100;
    sub_direct = nh.subscribe("/vehicle_status", 1, &MPCSaveTraj::callbackStatus,this);
    //ros::Subscriber sub_odom = nh.subscribe("/odom", 1, &MPCSaveTraj::callbackOdom,this);
    // ROS_INFO_STREAM("11!");
    sub_pose = nh.subscribe("/gnss_pose",1,&MPCSaveTraj::callbackGnssPose,this);
    // ROS_INFO_STREAM("c12!");
	//ros::Publisher trans_waypoints_pub = n.advertise<autoware_msgs::Lane>("/trans_debug_mpc", 1);
    };
    ~MPCSaveTraj(){};

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_pose;
    ros::Subscriber sub_direct;
    mpc_msgs::Lane debug_mpc_waypoints;
    geometry_msgs::PoseStamped veh_pose;
    mpc_msgs::Lane mpc_traj;
    mpc_msgs::VehicleStatus status;
    mpc_msgs::Waypoint waypoint_temp;
    geometry_msgs::PoseStamped front_pose;
    uint32_t count1 = 0; 
    uint32_t count2 = 0;
	uint32_t count3 = 0;



void callbackGnssPose(const geometry_msgs::PoseStamped &msg){
    veh_pose = msg;
    // ROS_INFO_STREAM("callbackGnsspose succceed!");

    auto sq_dist = [](const geometry_msgs::Point &a, const geometry_msgs::Point &b) 
    {
      const double dx = a.x - b.x;
      const double dy = a.y - b.y;
      return dx * dx + dy * dy;
    };
    if(sq_dist(veh_pose.pose.position,front_pose.pose.position)>=1){

            count2 = veh_pose.header.seq;
            waypoint_temp.direction = -1;
            if(count1 != count2)
            {
                count1 = count2;
                if(status.gear == 1)  //D档，前进
                {
                    waypoint_temp.direction = 0;
                }
                else if(status.gear == 2)  //R档，后退
                {
                    waypoint_temp.direction = 3;
                }
                else{
                    waypoint_temp.direction = 6; //D档或P档
                }

                waypoint_temp.pose = veh_pose;
                // debug_mpc_waypoints.waypoints.push_back(waypoint_temp);

                mpc_traj.header.seq = count3;
                // trans_waypoints_pub.publish(debug_mpc_waypoints);
                waypoint_temp.twist.twist.linear.x = 3;

                count3++;
                std::ofstream out("mpc_traj.txt", std::ios::app);
                out << waypoint_temp.pose.pose.position.x <<' '<<waypoint_temp.pose.pose.position.y<<' '<<waypoint_temp.pose.pose.position.z<<' '<< waypoint_temp.pose.pose.orientation.x<<' '<<waypoint_temp.pose.pose.orientation.y<<' '<<waypoint_temp.pose.pose.orientation.z<<' '<<waypoint_temp.pose.pose.orientation.w<<' '<<waypoint_temp.direction<<' ' <<waypoint_temp.twist.twist.linear.x<< std::endl;
                // ROS_INFO("%lf",waypoint_temp.pose.pose.position.x);
                out.close();
                // ROS_INFO_STREAM("WRITE succceed!");
                front_pose.pose.position = veh_pose.pose.position;
                // veh_pose.pose.position = fsubscriberont_pose.pose.position;
            }
    	
    }
}

void callbackStatus(const mpc_msgs::VehicleStatus &msg)
{
    status = msg;
}

};


int main(int argc,char *argv[]){
    ros::init(argc, argv, "mpc_save_traj");

    ROS_INFO_STREAM("initial!");
    MPCSaveTraj obj;
    // ROS_INFO_STREAM("aaaaa!");
    ros::spin();
    // ROS_INFO_STREAM("ALL FINISHED!");
    return 0;
}