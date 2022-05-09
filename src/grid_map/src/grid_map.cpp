#include "grid_map.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
static std::chrono::time_point<std::chrono::system_clock> process_start, process_end;

void GridMap::CloudCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud)
{
    process_start = std::chrono::system_clock::now();
    ROS_INFO("Point Cloud size:%d", in_sensor_cloud->height * in_sensor_cloud->width);
    input_points_.clear();
    input_points_.shrink_to_fit();
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*in_sensor_cloud, *point_cloud);
    for (size_t i = 0; i < point_cloud->size(); ++i)
    {
        Point3FI tmp;
        // Convert the read point cloud data to cm units
        tmp.x = 100 * point_cloud->points[i].x;
        if (tmp.x < min_x_ || tmp.x > max_x_)
            continue;
        tmp.y = 100 * point_cloud->points[i].y;
        if (tmp.y < min_y_ || tmp.y > max_y_)
            continue;
        tmp.z = 100 * point_cloud->points[i].z;
        if (tmp.z < min_z_ || tmp.z > max_z_)
            continue;

        // remove car points
        if (tmp.y > car_right_ && tmp.y < car_left_ && tmp.x > car_back_ && tmp.x < car_front_)
            continue;

        tmp.intensity = point_cloud->points[i].intensity;
        input_points_.push_back(tmp);
    }
    ROS_INFO("Points Number in Area:%ld", input_points_.size());

    // Assign points to the grid where they are located
    fillGrid();
    // Calculated obstacle grid
    calGrid();
    publish(grid_map_pub_);
}

void GridMap::fillGrid()
{
    //清空之前数据
    for (size_t i = 0; i < all_points_.size(); ++i)
    {
        all_points_[i].clear();
        all_points_[i].shrink_to_fit();
    }
    //计算
    for (size_t i = 0; i < input_points_.size(); ++i)
    {
        int x_id = getXindex(input_points_[i].x);
        int y_id = getYindex(input_points_[i].y);
        // in grid range
        if (-1 == x_id || -1 == y_id)
            continue;
        PointIndex tmp_p;
        // round 四舍五入
        tmp_p.x = round(input_points_[i].x);
        tmp_p.y = round(input_points_[i].y);
        tmp_p.z = round(input_points_[i].z);
        tmp_p.x_id = x_id;
        tmp_p.y_id = y_id;

        all_points_[x_id * grid_size_y_ + y_id].push_back(tmp_p);
    }
    //去除悬挂障碍物  remove hang obs
    // Leaves and other obstacles that would originally be judged as obstacles
    // but do not affect the operation of the vehicle, hang obstacles
    int window_size = 2;
    for (int i = 0; i < grid_point_.size(); i++)
    {
        grid_point_[i].point.clear();
        grid_point_[i].point.shrink_to_fit();
    }

    for (size_t x_id = 0; x_id < grid_size_x_; x_id++)
    {
        for (size_t y_id = 0; y_id < grid_size_y_; y_id++)
        {
            std::vector<PointIndex> sort_points;
            for (int i = -window_size; i <= window_size; i++)
            {
                for (int j = -window_size; j <= window_size; j++)
                {
                    if (x_id + i < 0 || x_id + i >= grid_size_x_ || y_id + j < 0 || y_id + j >= grid_size_y_)
                        continue;
                    if (all_points_[(x_id + i) * grid_size_y_ + y_id + j].empty())
                        continue;
                    else
                        //从sort_points 后端插入
                        sort_points.insert(sort_points.end(), all_points_[(x_id + i) * grid_size_y_ + y_id + j].begin(),
                                           all_points_[(x_id + i) * grid_size_y_ + y_id + j].end());
                }
            }
            if (sort_points.empty())
                continue;
            // 根据点的高度 从低到高
            sort(sort_points.begin(), sort_points.end(),
                 [](PointIndex a, PointIndex b)
                 { return a.z < b.z; });

            int smaller_z = 0;
            int larger_z = 0;
            int con_z_gap = 0;
            int max_id = sort_points.size() - 1;
            int min_id = 0;
            for (int k = 0; k < sort_points.size() - 1; k++)
            {
                // ？？
                larger_z = sort_points[k + 1].z;
                smaller_z = sort_points[k].z;
                con_z_gap = larger_z - smaller_z;
                if (con_z_gap > hang_z_gap_)
                {
                    if (fabs((double)larger_z) > fabs((double)smaller_z))
                        max_id = k;
                    else
                        min_id = k + 1;
                    break;
                }
            }
            for (int k = min_id; k <= max_id; k++)
            {
                // bug
                if (sort_points[k].x_id != x_id - window_size || sort_points[k].y_id != y_id - window_size)
                    continue;
                Point3II tmp;
                tmp.x = sort_points[k].x;
                tmp.y = sort_points[k].y;
                tmp.z = sort_points[k].z;
                tmp.intensity = 255;
                grid_point_[(sort_points[k].x_id) * grid_size_y_ + (sort_points[k].y_id)].point.push_back(tmp);
                grid_point_[(sort_points[k].x_id) * grid_size_y_ + (sort_points[k].y_id)].is_full = 1;
            }
            sort_points.clear();
            sort_points.shrink_to_fit();
        }
    }
}

int GridMap::getXindex(double x)
{
    int x_index = round(x / resolution_x_) + offset_x_;
    if (x_index >= grid_size_x_ || x_index < 0)
        return -1;
    else
        return x_index;
}

int GridMap::getYindex(double y)
{
    int y_index = round(y / resolution_y_) + offset_y_;
    if (y_index >= grid_size_y_ || y_index < 0)
        return -1;
    else
        return y_index;
}

void GridMap::calGrid()
{
    for (size_t i = 0; i < grid_point_.size(); i++)
    {
        if (grid_point_[i].point.empty())
        {
            grid_label_[i].pos_obs = 0;
            inv_obs_grid_[i] = 255;
            continue;
        }
        else
        {
            grid_value_[i].count = grid_point_[i].point.size();
            grid_value_[i].filled = 1;
            grid_value_[i].sum = 0;
            grid_value_[i].z_max = -10000;
            grid_value_[i].z_min = 10000;
            double zz_sum = 0;
            // int line_num = 0;
            for (int j = 0; j < grid_point_[i].point.size() - 1; j++)
            {
                double z_p = grid_point_[i].point[j].z;
                grid_value_[i].sum += z_p;
                zz_sum += z_p * z_p;
                if (z_p > grid_value_[i].z_max)
                    grid_value_[i].z_max = z_p;
                if (z_p < grid_value_[i].z_min)
                    grid_value_[i].z_min = z_p;

                // Point3II low = grid_point_[i].point[j];
                // Point3II high = grid_point_[i].point[j+1];
                // double angle_low = atan( fabs((double)low.z)/hypot(fabs((double)low.x),fabs((double)low.y)) );
                // double angle_high = atan( fabs((double)high.z)/hypot(fabs((double)high.x),fabs((double)high.y)) );
                // if(angle_high - angle_low>0.00523)
                //   line_num++;
            }
            // if(line_num > 1)
            // {
            //   grid_label_[i].pos_obs = 1;
            //   inv_obs_grid_[i] = 0;
            //   continue;
            // }
            // Check if it is an obstacle
            grid_value_[i].z_mean = grid_value_[i].sum / grid_value_[i].count;
            grid_value_[i].z_idw = zz_sum / grid_value_[i].count - grid_value_[i].z_mean * grid_value_[i].z_mean;
            // Calculates the smoothness of the height of points, one of which is to eliminate occasional noise points of varying heights
            if (grid_value_[i].z_max - grid_value_[i].z_min > hd_threshold_ && grid_value_[i].z_idw > rough_threshold_)
            {
                inv_obs_grid_[i] = 0;
                grid_label_[i].pos_obs = 1;
            }
            else
            {
                grid_label_[i].pos_obs = 0;
                inv_obs_grid_[i] = 255;
            }
        }
    }
}

void GridMap::publish(ros::Publisher pub)
{
    nav_msgs::OccupancyGrid map;
    map.header.frame_id = base_frame_;
    map.header.stamp = ros::Time::now();
    map.info.resolution = resolution_x_ / 100.0;
    map.info.width = grid_size_x_;
    map.info.height = grid_size_y_;
    map.info.origin.position.x = min_x_ / 100.0;
    map.info.origin.position.y = min_y_ / 100.0;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.x = 0.0;
    map.info.origin.orientation.y = 0.0;
    map.info.origin.orientation.z = 0.0;
    map.info.origin.orientation.w = 1.0;
    unsigned char res_grid[51200];
    memset(res_grid, 0, 51200);

    for (int i = 0; i < grid_size_x_; ++i)
    {
        for (int j = 0; j < grid_size_y_; ++j)
        {
            int idx = i * grid_size_y_ + j;
            int idx2 = j * grid_size_x_ + i;
            if (grid_label_[idx].pos_obs)
            {
                res_grid[idx2] = 255;
            }
            else
            {
                res_grid[idx2] = 0;
            }
        }
    }
    // erode and dilate
    cv::Mat img(grid_size_y_, grid_size_x_, CV_8UC1, (unsigned char *)res_grid);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(img, img, element);

    cv::erode(img, img, element);

    std::vector<signed char> a(51200);
    for (size_t nrow = 0; nrow < grid_size_y_; nrow++)
    {
        for (size_t ncol = 0; ncol < grid_size_x_; ncol++)
        {
            int idx = ncol + nrow * grid_size_x_;
            if (img.at<uchar>(nrow, ncol) == 255)
                a[idx] = 100;
            else
                a[idx] = 0;
        }
    }
    for (size_t nrow = 1; nrow < grid_size_y_ - 1; nrow++)
    {
        for (size_t ncol = 1; ncol < grid_size_x_ - 1; ncol++)
        {
            int idx = ncol + nrow * grid_size_x_;
            int left = ncol - 1 + nrow * grid_size_x_;
            int right = ncol + 1 + nrow * grid_size_x_;
            int up = ncol + (nrow - 1) * grid_size_x_;
            int down = ncol + (nrow + 1) * grid_size_x_;
            if (a[left] == 0 && a[right] == 0 && a[up] == 0 && a[down] == 0)
                a[idx] = 0;
        }
    }

    map.data = a;
    // map.data = img.data;
    pub.publish(map);
}

GridMap::GridMap() : nh_(), pnh_("~"), tf_listener_(tf_buffer_)
{
}

void GridMap::Run()
{
    ROS_INFO("Initializing Obstace Gridmap Node, please wait...");
    pnh_.param<std::string>("input_point_topic", input_point_topic_, "points_no_ground");
    ROS_INFO("Input point_topic: %s", input_point_topic_.c_str());

    pnh_.param<std::string>("base_frame", base_frame_, "rslidar");
    ROS_INFO("base_frame: %s", base_frame_.c_str());

    std::string grid_topic;
    pnh_.param<std::string>("grid_topic", grid_topic, "points_ground");
    ROS_INFO("Publisht Gridmap Topic: %s", grid_topic.c_str());

    // pnh_.param<std::string>("frame_id", frame_id_, "/rslidar");

    // The setting range, the unit is cm
    pnh_.param<double>("min_x", min_x_, -2000.0);
    pnh_.param<double>("max_x", max_x_, 6000.0);
    pnh_.param<double>("min_y", min_y_, -2000.0);
    pnh_.param<double>("max_y", max_y_, 2000.0);
    pnh_.param<double>("min_z", min_z_, -200.0);
    pnh_.param<double>("max_z", max_z_, 100.0);
    // Range of vehicle platforms

    pnh_.param<double>("car_left", car_left_, 50.0);
    pnh_.param<double>("car_right", car_right_, -50.0);
    pnh_.param<double>("car_front", car_front_, 10.0);
    pnh_.param<double>("car_back", car_back_, -120.0);
    // resolution of the map 320×160
    pnh_.param<int>("grid_size_x", grid_size_x_, 320);
    pnh_.param<int>("grid_size_y", grid_size_y_, 160);
    pnh_.param<int>("resolution_x", resolution_x_, 25); // Physical distance per grid, the unit is cm,
    pnh_.param<int>("resolution_y", resolution_y_, 25);
    pnh_.param<int>("offset_x", offset_x_, 80); // 2000/25, make the range of all values positive
    pnh_.param<int>("offset_y", offset_y_, 80);

    pnh_.param<double>("hang_z_gap", hang_z_gap_, 80.0);
    pnh_.param<double>("hd_threshold", hd_threshold_, 15.0);
    pnh_.param<double>("rough_threshold", rough_threshold_, 20.0);

    ROS_INFO("Area Range: x:%lf-%lf,y:%lf-%lf,z:%lf-%lf", min_x_, max_x_, min_y_, max_y_, min_z_, max_z_);

    all_points_.resize(grid_size_x_ * grid_size_y_);
    grid_point_.resize(grid_size_x_ * grid_size_y_);
    grid_value_.resize(grid_size_x_ * grid_size_y_);
    grid_label_.resize(grid_size_x_ * grid_size_y_);
    all_points_.resize(grid_size_x_ * grid_size_y_);
    inv_obs_grid_.resize(grid_size_x_ * grid_size_y_);

    ROS_INFO("Subscribing to... %s", input_point_topic_.c_str());
    points_node_sub_ = nh_.subscribe(input_point_topic_, 1, &GridMap::CloudCallback, this);

    grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(grid_topic, 2);

    ROS_INFO("Ready");
    ros::spin();
}