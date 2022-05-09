#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <mpc_msgs/Lane.h>
#include <mpc_msgs/VehicleStatus.h>
#include <mpc_msgs/Waypoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>

class MPCReadTraj{
public:
    MPCReadTraj(){
        ROS_INFO_STREAM("initial here");
        //发布局部路径信息（只取最近路径点前后的若干点，不发布所有的路径信息）
        pub_waypoints_ = nh.advertise<mpc_msgs::Lane>("/mpc_waypoints",1); 
        ROS_INFO_STREAM("start read");
        Read_traj();
        ROS_INFO_STREAM("read all again");
        sub_current_pose_ = nh.subscribe("/gnss_pose",1,&MPCReadTraj::callbackconverter,this);
        closest_idx_ = 0;  
        back_waypoints_num_ = 10;
        front_waypoints_num_ = 50;
    };
    ~MPCReadTraj(){};
private:
    ros::NodeHandle nh;
    geometry_msgs::PoseStamped current_pose;
    int closest_idx_;
    int back_waypoints_num_;
    int front_waypoints_num_;
    mpc_msgs::Lane base_waypoints_;
    ros::Publisher pub_waypoints_;
    ros::Subscriber sub_current_pose_;
    

void Read_traj( ){
    std::ifstream myfile("mpc_traj.txt");
    if(myfile.fail()){
        ROS_INFO("read failed");
        return ;
    }
    while(!myfile.eof()){
        char buffer[1000];
        mpc_msgs::Waypoint temp_waypoint;
        myfile.getline(buffer,1000);
        if(myfile.eof()) break;
        sscanf(buffer,"%lf %lf %lf %lf %lf %lf %lf %d %lf",&temp_waypoint.pose.pose.position.x,&temp_waypoint.pose.pose.position.y,&temp_waypoint.pose.pose.position.z,&temp_waypoint.pose.pose.orientation.x,&temp_waypoint.pose.pose.orientation.y,&temp_waypoint.pose.pose.orientation.z,&temp_waypoint.pose.pose.orientation.w,&temp_waypoint.direction,&temp_waypoint.twist.twist.linear.x);
        // ROS_INFO("%lf %#include <nav_msgs/Odometry.h>lf %lf %lf %lf %lf %lf %d %lf",temp_waypoint.pose.pose.position.x,temp_waypoint.pose.pose.position.y,temp_waypoint.pose.pose.position.z,temp_waypoint.pose.pose.orientation.x,temp_waypoint.pose.pose.orientation.y,temp_waypoint.pose.pose.orientation.z,temp_waypoint.pose.pose.orientation.w,temp_waypoint.direction,temp_waypoint.twist.twist.linear.x);

        base_waypoints_.waypoints.push_back(temp_waypoint);
        // final_waypoints.waypoints.push_back(temp_waypoint);
    }
    // ROS_INFO("%lf",base_waypoints_base_waypoints_.waypoints.size());
    ROS_INFO("%lf",base_waypoints_.waypoints.at(0).pose.pose.position.x);
    ROS_INFO_STREAM("read all");
}

void callbackconverter(const geometry_msgs::PoseStamped &msg){
    ROS_INFO_STREAM("start mpc");
    current_pose = msg;

    mpc_msgs::Lane mpc_waypoints;
    auto sq_dist = [](const geometry_msgs::Point &a, const geometry_msgs::Point &b) {
      const double dx = a.x - b.x;
      const double dy = a.y - b.y;
      return dx * dx + dy * dy;
    };

    //寻找最近点
    int closest_idx = -1;
    for (int i = 0; i < (int)base_waypoints_.waypoints.size(); ++i) {
      const double d = sq_dist(current_pose.pose.position, base_waypoints_.waypoints[i].pose.pose.position);
      if (d < 0.25) {
        closest_idx = i;
        break;
      }
    }
    if (closest_idx == -1) {
      ROS_ERROR("cannot find closest base_waypoints' waypoint to final_waypoints.waypoint[1] !!  ganlu");
    }

    ROS_INFO("%d",closest_idx);
    // 最近点之前的back_waypoints_num个路径点（已经跑过的）
    int base_start = std::max(closest_idx - back_waypoints_num_, 0);
    for (int i = base_start; i < closest_idx; ++i)
    {
      mpc_waypoints.waypoints.push_back(base_waypoints_.waypoints.at(i));
      // mpc_waypoints.waypoints.back().twist = final_waypoints.waypoints[1].twist;
      mpc_waypoints.waypoints.back().twist = base_waypoints_.waypoints[closest_idx].twist;
    }

    // 最近点之后的front_waypoints_num_个路径点（还没跑过的）
    int final_end = std::min(front_waypoints_num_ + closest_idx, (int)base_waypoints_.waypoints.size());
    for (int i = closest_idx; i < final_end; ++i)
    {
      mpc_waypoints.waypoints.push_back(base_waypoints_.waypoints.at(i));
    }
    pub_waypoints_.publish(mpc_waypoints);

}
};



int main(int argc,char *argv[]){
    ROS_INFO_STREAM("code start");
    ros::init(argc, argv, "mpc_read_traj");   
    MPCReadTraj obj;
    ros::spin();
    return 0;
}