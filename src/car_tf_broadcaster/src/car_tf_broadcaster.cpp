
#include "car_tf_broadcaster.h"
void CarTF::callbackGnssPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    _gnss_pose = *msg;
}

void CarTF::callbackTimerPublishTF(const ros::TimerEvent &e)
{
    publishTF();
}

void CarTF::publishTF()
{
    //广播map->base_link和base_link->lidar_rs的tf

    // map->base_link 
    geometry_msgs::TransformStamped base_link_transform;

    base_link_transform.header.frame_id = map_frame;
    base_link_transform.header.stamp = ros::Time::now();
 
    base_link_transform.child_frame_id = base_link_frame;

    base_link_transform.transform.translation.x = _gnss_pose.pose.position.x;
    base_link_transform.transform.translation.y = _gnss_pose.pose.position.y;
    base_link_transform.transform.translation.z = _gnss_pose.pose.position.z;
    base_link_transform.transform.rotation = _gnss_pose.pose.orientation;

    _tf_broadcaster.sendTransform(base_link_transform);
  
    // base_link->lidar_rs
    geometry_msgs::TransformStamped lidar_transform;
 
    lidar_transform.header.frame_id = base_link_frame;
    lidar_transform.header.stamp = ros::Time::now();

    lidar_transform.child_frame_id = lidar_frame;

    lidar_transform.transform.translation.x = lidar_trans_x;
    lidar_transform.transform.translation.y = lidar_trans_y;
    lidar_transform.transform.translation.z = lidar_trans_z;

    geometry_msgs::Quaternion q;
    q = tf::createQuaternionMsgFromRollPitchYaw(lidar_rotation_roll,
                                                lidar_rotation_pitch,
                                                lidar_rotation_yaw);
    lidar_transform.transform.rotation.w = q.w;
    lidar_transform.transform.rotation.x = q.x;
    lidar_transform.transform.rotation.y = q.y;
    lidar_transform.transform.rotation.z = q.z;

    _tf_broadcaster.sendTransform(lidar_transform);

    //广播结束
}

CarTF::CarTF() : _nh(""), _private_nh("~")
{

    _private_nh.param<double>("lidar_trans_x", lidar_trans_x, 4.34);
    _private_nh.param<double>("lidar_trans_y", lidar_trans_y, 0.58);
    _private_nh.param<double>("lidar_trans_z", lidar_trans_z, 0.0);
    _private_nh.param<double>("lidar_rotation_roll", lidar_rotation_roll, 0.0);
    _private_nh.param<double>("lidar_rotation_pitch", lidar_rotation_pitch, 0.0);
    _private_nh.param<double>("lidar_rotation_yaw", lidar_rotation_yaw, 0.0);
    _private_nh.param<std::string>("map_frame", map_frame, "map");
    _private_nh.param<std::string>("base_link_frame", base_link_frame, "base_link");
    _private_nh.param<std::string>("lidar_frame_id", lidar_frame, "rslidar");
    _private_nh.param<std::string>("pose_topic", pose_topic, "gnss_pose");
    _private_nh.param<double>("dt", dt, 0.1);

    sub_gnss_pose = _nh.subscribe<geometry_msgs::PoseStamped>(pose_topic, 1, &CarTF::callbackGnssPose, this);

    timer_tf = _nh.createTimer(ros::Duration(dt), &CarTF::callbackTimerPublishTF, this);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_tf_broadcaster");

    CarTF obj;

    // ros::Rate loop_rate(50);
    // while (ros::ok())
    // {
    //     ros::spinOnce();

    //     loop_rate.sleep();
    // }
    ros::spin();

    return 0;
}