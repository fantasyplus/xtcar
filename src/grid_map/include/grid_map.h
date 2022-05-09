#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <iostream>
#include <algorithm>
#include <vector>
#include <memory>
#include <string>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
// #include <velodyne_pointcloud/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include "common.h"

class GridMap
{
private:
  // node handler
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  // suber puber
  ros::Subscriber points_node_sub_;
  ros::Publisher grid_map_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string input_point_topic_;
  std::string base_frame_;

  std::vector<Point3FI> input_points_;
  std::vector<std::vector<PointIndex>> all_points_;
  std::vector<GridPoint> grid_point_;
  std::vector<GridValue> grid_value_;
  std::vector<GridLabel> grid_label_;
  std::vector<double> inv_obs_grid_;
  
  // 感兴趣区域范围
  double min_x_;
  double max_x_;//前方
  double min_y_;
  double max_y_;//左右
  double min_z_;
  double max_z_;//高度方向
  // 车体范围
  double car_left_;
  double car_right_; //左右
  double car_front_;
  double car_back_;//前后
  //grid 设置
  int grid_size_x_;
  int grid_size_y_;
  int resolution_x_;
  int resolution_y_;
  int offset_x_;
  int offset_y_;
  // 阈值设置
  double hang_z_gap_;
  double hd_threshold_;
  double rough_threshold_;


  void fillGrid();
  void calGrid();
  int getXindex(double x);
  int getYindex(double y);

  void CloudCallback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud);
  void publish(ros::Publisher pub);
public:
  GridMap();
  void Run();
};
#endif  //GRID_MAP_H