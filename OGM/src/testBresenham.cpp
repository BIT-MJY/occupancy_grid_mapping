#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <occupancy_grid_mapping/Map.h>
#include <thread>
#include <mutex>
#include <chrono>
#include<occupancy_grid_mapping/rayCaster.h>
#include"rayCaster.cpp"
#include<occupancy_grid_mapping/Map.h>
#include"Map.cpp"


struct Point{

double x;
double y;
double odom_x;
double odom_y;
double range;
double angle;
ros::Time time;


};




class Tester{
public:
    Tester(int size_x, int size_y, double resolution, tf::TransformListener *tf);
    void convert2world(std::vector<Point>& points,  tf::TransformListener *tf_);
    
    std::vector<Point> points;
    ros::Subscriber laser_sub_;
    ros::Publisher marker_pub;
    
    Map* map;
    Caster caster;
    
    int size_x_;
    int size_y_;
    double resolution_;
    tf::TransformListener *tf_;
    ros::NodeHandle n;
    
    int flag = 0;
    int cnt = 0;
    
    int base_x;
    int base_y;
    
    std::mutex valMutex;
    std::mutex valMutex2;
    
    void laserCallback(const sensor_msgs::LaserScan &laser_msgs);
    bool getLaserPose(tf::Stamped<tf::Pose> &odom_pose, ros::Time dt, tf::TransformListener * tf_);
    void showThisFrame();
    void showPoint(double& x, double& y);
    void showLine();

    
};



Tester::Tester(int size_x, int size_y, double resolution, tf::TransformListener *tf):
                                                size_x_(size_x),size_y_(size_y), resolution_(resolution),tf_(tf){
        std::cout<<"tester is ready"<<std::endl;
        this->laser_sub_ = this->n.subscribe("/scan",10, &Tester::laserCallback,this);
        this->marker_pub = this->n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        this->map = new Map(size_x_, size_y_, resolution);

        }



void Tester::laserCallback(const sensor_msgs::LaserScan &laser_msgs){
    
    if (cnt++ % 1 != 0)  // 控制回调频率
    {
        return;
    }
    
    std::lock_guard<std::mutex> lg(this->valMutex);

    
    ros::Time startTime, endTime;
    // start time
    startTime = laser_msgs.header.stamp;

    // end time 
    // number of points
    int beamNum = laser_msgs.ranges.size();
    endTime = startTime + ros::Duration(laser_msgs.time_increment * beamNum);
    // loop over laser
//     std::cout<<"111111111111"<<std::endl;
    for (int i=0; i<beamNum; i++){

        Point point;

        // ===============================================get Point in the world=========================================================
//         tf::Stamped<tf::Pose> visualPose;
//         // ros time of this point
//         ros::Time point_time = startTime + ros::Duration(laser_msgs.time_increment * i);
//         point.time = point_time;
//         // laser_link in odom  -----  visualPose
//         if (!this->getLaserPose(visualPose, point_time, this->tf_)){
//             ROS_WARN("Not visualPose!!!!!!");
//             return;
//         }
//         double visualYaw = tf::getYaw(visualPose.getRotation());
        point.range = laser_msgs.ranges[i]; 
        point.angle = laser_msgs.angle_min + laser_msgs.angle_increment * i;
//         double tmp = (startTime + ros::Duration(laser_msgs.time_increment * i)).toSec() * 1000 * 1000;
//         point.time = ros::Time(tmp/1000000.0);
        point.time = startTime + ros::Duration(laser_msgs.time_increment * i);
//         std::cout<<point.time<<std::endl;
        this->points.push_back(point);
    }
    
    convert2world(this->points, this->tf_);
//         std::cout<<"33333333333333333"<<std::endl;
    
    showThisFrame();
    
    this->points.clear();
    

}



void Tester::convert2world(std::vector<Point> &points,  tf::TransformListener *tf_)
{

    for (auto& point:points){
        if (isinf( point.range ))
        {
//             break;
            point.range = 0;
        }
        tf::Stamped<tf::Pose> odom_pose;
//         std::cout<<point.time<<std::endl;
        if (!getLaserPose(odom_pose, point.time, tf_))
        {
            ROS_ERROR("无法捕获odom与baselink的转换关系");
        }
        tf::Quaternion base_in_odom_angle_q = odom_pose.getRotation();
        double base_in_odom_angle = tf::getYaw(base_in_odom_angle_q);
        tf::Vector3 base_in_odom_pos = odom_pose.getOrigin();
//         std::cout<<base_in_odom_pos.x()<<", "<<base_in_odom_pos.y()<<std::endl;
        // 激光雷达坐标系下的坐标
        double laser_x, laser_y;
        laser_x = point.range * cos(point.angle);
        laser_y = point.range * sin(point.angle);
        // 里程计坐标系下的坐标
        // 之前的tf::transform的第一个参数为“odom”，证明得到的变换是将其他坐标系的点转换到odom系中的变换
        // double odom_x, odom_y;
        point.odom_x = laser_x * cos(base_in_odom_angle) - laser_y * sin(base_in_odom_angle) + base_in_odom_pos.x();
        point.odom_y = laser_x * sin(base_in_odom_angle) + laser_y * cos(base_in_odom_angle) + base_in_odom_pos.y();
        
        // TODO:===================================START===多线程未完成==============================================
        
        std::pair<int, int> point_cell_xy = map->getCellIndex(point.odom_x, point.odom_y);
        std::pair<int, int> base_cell_xy = map->getCellIndex(base_in_odom_pos.x(), base_in_odom_pos.y());
//         
        this->caster.Bresenham(base_cell_xy.first, base_cell_xy.second,point_cell_xy.first, point_cell_xy.second, this->caster.grid_index_vec);
//         
//         // 展示划线效果
//         std::thread t1([=](){this->showLine();});
//         t1.detach();
        this->showLine();
//         // 将索引返还
//         for (auto &grid: (this->caster.grid_index_vec))
//         {
//             std::pair<double,double> grid_ =  this->map->fromCellIndex(grid.first,grid.second);
//             
//             
//             // 如果直接展示线程会发生堵塞，因此必须多线程
// //             this->showPoint(grid_.first,grid_.second);
//             // lambda表达式
//         }
//         t1.join();
//         this->caster.grid_index_vec.clear();
        // TODO:========================================================================================

    }
    
}


bool Tester::getLaserPose(tf::Stamped<tf::Pose> &odom_pose, ros::Time dt, tf::TransformListener * tf_)
{
    
        odom_pose.setIdentity();

        tf::Stamped < tf::Pose > robot_pose;
        robot_pose.setIdentity();
        // lidar_link
        robot_pose.frame_id_ = "base_link";
        robot_pose.stamp_ = dt;   //设置为ros::Time()表示返回最近的转换关系
//         std::cout<<dt<<std::endl;

        // get the global pose of the robot
        try
        {
            // ROS_INFO("debug!!!");
            if(!tf_->waitForTransform("/odom", "/base_link", dt, ros::Duration(0.5)))             // 0.15s 的时间可以修改
            {
                ROS_ERROR("LidarMotion-Can not Wait Transform()");
                return false;
            }
            // data of lidar_link is transformed to odom coordinate 
            // the result is in the odom_pose
            tf_->transformPose("/odom", robot_pose, odom_pose);
    
        }
        catch (tf::LookupException& ex)
        {
            ROS_ERROR("LidarMotion: No Transform available Error looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_ERROR("LidarMotion: Connectivity Error looking up looking up robot pose: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException& ex)
        {
            ROS_ERROR("LidarMotion: Extrapolation Error looking up looking up robot pose: %s\n", ex.what());
            return false;
        }
        return true;
}

void Tester::showThisFrame(){
    for (auto& point:points){
    
        this->showPoint(point.odom_x, point.odom_y);
    }
    
    
    
}

void Tester::showPoint(double& x, double& y)
{
    uint32_t shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = this->flag++;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(5);  // not to disappear
    marker_pub.publish(marker);
}

void Tester::showLine()
{
//         std::lock_guard<std::mutex> lg(this->valMutex2);
    int c = 0;
    for (auto &grid: (this->caster.grid_index_vec))
        {
            if (c++%10 == 0)
            {
                std::pair<double,double> grid_ =  this->map->fromCellIndex(grid.first,grid.second);
                this->showPoint(grid_.first,grid_.second);   
            }

        }
//         this->caster.grid_index_vec.clear();
    std::vector < std::pair<int,int>>().swap(this->caster.grid_index_vec);

}

void getParamFromYaml(int &map_size_x, int &map_size_y, double &map_resolution, bool &calib)
{
    ros::param::get("~map_size_x_",map_size_x);  // x方向栅格数目
    ros::param::get("~map_size_y_",map_size_y);  // y方向栅格数目
    ros::param::get("~resolution_",map_resolution);  // m/格子
    ros::param::get("~calib",calib);  // 是否进行激光雷达数据矫正
}

int main (int argc, char** argv) {

    ros::init(argc, argv, "ogm_test_node");
    int map_size_x, map_size_y;
    double map_resolution;
    bool calib;
    getParamFromYaml(map_size_x, map_size_y, map_resolution, calib);
    tf::TransformListener tf(ros::Duration(10.0));
    // 300*300的栅格，分辨率为0.1m/格
    // MapBuilder map_builder(300, 300, 0.1, &tf);
    Tester tester(map_size_x, map_size_y, map_resolution, &tf);
    ros::spin();

}
