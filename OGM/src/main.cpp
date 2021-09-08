#include <Eigen/Core>
#include <math.h>
// #include "occupancy_grid_mapping/MapBuilder.h"
// #include "occupancy_grid_mapping/Map.h"
#include "MapBuilder.cpp"
#include "Map.cpp"
#include "rayCaster.cpp"

void getParamFromYaml(int &map_size_x, int &map_size_y, double &map_resolution, bool &calib)
{
    ros::param::get("~map_size_x_",map_size_x); 
    ros::param::get("~map_size_y_",map_size_y); 
    ros::param::get("~resolution_",map_resolution);  
    ros::param::get("~calib",calib); 
}



int main (int argc, char** argv) {
    ros::init(argc, argv, "ogm_node");
    int map_size_x, map_size_y;
    double map_resolution;
    bool calib;
    getParamFromYaml(map_size_x, map_size_y, map_resolution, calib);
    tf::TransformListener tf(ros::Duration(10.0));
    MapBuilder map_builder(map_size_x, map_size_y, map_resolution, &tf, calib);
    ros::spin();
}
