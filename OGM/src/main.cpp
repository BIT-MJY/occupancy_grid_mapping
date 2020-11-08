#include <Eigen/Core>
#include <math.h>
// #include "occupancy_grid_mapping/MapBuilder.h"
// #include "occupancy_grid_mapping/Map.h"
#include "MapBuilder.cpp"
#include "Map.cpp"
#include "rayCaster.cpp"

void getParamFromYaml(int &map_size_x, int &map_size_y, double &map_resolution, bool &calib)
{
    ros::param::get("~map_size_x_",map_size_x);  // x方向栅格数目
    ros::param::get("~map_size_y_",map_size_y);  // y方向栅格数目
    ros::param::get("~resolution_",map_resolution);  // m/格子
    ros::param::get("~calib",calib);  // 是否进行激光雷达数据矫正
}



int main (int argc, char** argv) {
    ros::init(argc, argv, "ogm_node");
    int map_size_x, map_size_y;
    double map_resolution;
    bool calib;
    getParamFromYaml(map_size_x, map_size_y, map_resolution, calib);
    tf::TransformListener tf(ros::Duration(10.0));
    // 300*300的栅格，分辨率为0.1m/格
    // MapBuilder map_builder(300, 300, 0.1, &tf);
    MapBuilder map_builder(map_size_x, map_size_y, map_resolution, &tf, calib);
    ros::spin();
}
