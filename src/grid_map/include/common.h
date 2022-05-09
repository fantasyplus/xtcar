#ifndef COMMON_H
#define COMMON_H

#include <vector>
struct Point3FI
{
    double x;
    double y;
    double z;
    unsigned char intensity;
};

struct PointIndex
{
    int x;
    int y;
    int z;
    int x_id;
    int y_id;
};

struct Point3II
{
    int x;
    int y;
    int z;
    unsigned char intensity;
};

struct GridValue
{
    double z_min;
    double z_max;
    double z_mean;
    int count;
    double z_idw;
    double sum;
    int filled;
};

struct GridPoint
{
    std::vector<Point3II> point;
    int min_z_label;
    int max_z_label;
    char is_full;
    char is_hanging_obs;
};


struct GridLabel
{
    char safe_area;
    char pos_obs;
    char neg_obs;
    char water_area;
    char shadow_area;
    char dynamic_area;
    char hang_area;
    char cliff_area;
    double safe_prob;
};

#endif // COMMON_H