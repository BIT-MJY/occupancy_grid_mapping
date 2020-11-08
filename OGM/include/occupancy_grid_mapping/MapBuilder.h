#pragma once
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <occupancy_grid_mapping/Map.h>
// #include <thread>
# include <mutex>

struct Point{

double x;
double y;
double range;
double angle;
ros::Time time;

};


class MapBuilder{

public:
    MapBuilder(int map_size_x, int map_size_y, double resolution, tf::TransformListener *tf, bool cal);
            
        
    void odomCallback(const nav_msgs::Odometry &odom_msgs);
    void laserCallback(const sensor_msgs::LaserScan &laser_msgs);
    bool getLaserPose(tf::Stamped<tf::Pose> &odom_pose, ros::Time dt, tf::TransformListener * tf_);
    // 激光矫正
    void laserCalib(std::vector<Point> points,ros::Time startTime, ros::Time endTime, tf::TransformListener * tf_);
    void laserMotionCalibration(tf::Stamped<tf::Pose> frame_base_pose,
                                                tf::Stamped<tf::Pose> frame_start_pose,
                                                tf::Stamped<tf::Pose> frame_end_pose,
                                                std::vector<Point> &points,
                                                int start_index,
                                                int beam_number  );

    ros::NodeHandle n;
    int map_size_x_;
    int map_size_y_;
    double resolution_;
    ros::Subscriber odom_sub_;
    ros::Subscriber laser_sub_;
    std::map<ros::Time, nav_msgs::Odometry> odom_list;
    std::vector<Point> points;
    tf::TransformListener* tf_;
    bool calib = true;
    Map* occ_map;
    
    
    std::mutex valMutex;


};
