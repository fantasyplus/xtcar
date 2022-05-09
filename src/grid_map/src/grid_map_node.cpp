#include <ros/ros.h>
#include "grid_map.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grid_map");

    GridMap map;
    map.Run();
    
    return 0;

}