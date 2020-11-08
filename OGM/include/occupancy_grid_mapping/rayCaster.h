#pragma once
#include <ros/ros.h>
#include <math.h>
#include <occupancy_grid_mapping/Map.h>
#include <Eigen/Dense>
#include <iostream>


class Map;

class Caster{
    
public:
    Caster();
    void updateMap(int x, int y, Eigen::MatrixXd& grid_map);
    void Bresenham(int &odom_x, int &odom_y, int &eigen_x, int &eigen_y, std::vector<std::pair<int,int>> &grid_index_vec);
    std::vector<std::pair<int,int>> grid_index_vec;
    
    int size_x;
    int size_y;
//     double resolution;
    
};
