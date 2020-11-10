#pragma once
#include <ros/ros.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <occupancy_grid_mapping/rayCaster.h>
#include <Eigen/Dense>  
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/core/core.hpp>        
#include <opencv2/highgui/highgui.hpp> 
#include <iostream>
#include <occupancy_grid_mapping/rayCaster.h>

class Map{

    
public:
    
    Map(int size_x_, int size_y, double resolution);
    
    int size_x;
    int size_y;
    double resolution;
    
    int flag = 0;
    int count2show = 0;
    
    
    ros::NodeHandle n_;
    ros::Publisher marker_pub;
    Caster caster;
    Eigen::MatrixXd grid_map; 
    bool calib;
    
    std::pair<int,int> getCellIndex(double x, double y);
    void rayCast(double x, double y, tf::Vector3 mid_pose);
    void showPoint(double x, double y);
    void showMap();
    cv::Mat normalize(cv::Mat srcImage);
};

    
