# include "occupancy_grid_mapping/MapBuilder.h"


MapBuilder::MapBuilder(int map_size_x, int map_size_y, double resolution, tf::TransformListener *tf, bool cal):
    map_size_x_(map_size_x),
    map_size_y_(map_size_y),
    resolution_(resolution),
    tf_(tf),
    calib(cal){
        std::cout<<"map builder is ready"<<std::endl;
//         this->odom_sub_ = this->n.subscribe("/odom",10, &MapBuilder::odomCallback,this);
        this->laser_sub_ = this->n.subscribe("/scan",10, &MapBuilder::laserCallback,this);

//         this->occ_map.size_x = map_size_x;
//         this->occ_map.size_y = map_size_y;
//         this->occ_map.resolution = resolution;
        this->occ_map = new Map(map_size_x_,map_size_y_,resolution_);
        this->occ_map->calib = calib;

    }
    

void MapBuilder::odomCallback(const nav_msgs::Odometry &odom_msgs){
    this->odom_list.insert(std::pair<ros::Time, nav_msgs::Odometry>(odom_msgs.header.stamp, odom_msgs));   // ros time is us
    if (this->odom_list.size() > 20)  // short memory
    {
        this->odom_list.erase(this->odom_list.begin());
    }
}


void MapBuilder::laserCallback(const sensor_msgs::LaserScan &laser_msgs){
    
    std::lock_guard<std::mutex> lg(this->valMutex);
    auto t1=std::chrono::steady_clock::now();
//     this->points.clear();
    std::vector <Point>().swap(this->points);
//     this->points.clear();
    
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
        point.time = startTime + ros::Duration(laser_msgs.time_increment * i);
        this->points.push_back(point);
//         std::cout<<"2222222222222222"<<std::endl;
    }
//         std::cout<<"33333333333333333"<<std::endl;
    MapBuilder::laserCalib(this->points, startTime, endTime, this->tf_);
    auto t2=std::chrono::steady_clock::now();
    std::cout<<"time for one frame:"<<std::chrono::duration<double,std::milli>(t2-t1).count()<<std::endl;

}

bool MapBuilder::getLaserPose(tf::Stamped<tf::Pose> &odom_pose, ros::Time dt, tf::TransformListener * tf_)
{
    
        odom_pose.setIdentity();

        tf::Stamped < tf::Pose > robot_pose;
        robot_pose.setIdentity();
        // lidar_link
        robot_pose.frame_id_ = "base_link";
        robot_pose.stamp_ = dt; 

        // get the global pose of the robot
        try
        {
            // ROS_INFO("debug!!!");
            if(!tf_->waitForTransform("/odom", "/base_link", dt, ros::Duration(0.5)))     
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


void MapBuilder::laserCalib(std::vector<Point> points,ros::Time startTime, ros::Time endTime, tf::TransformListener * tf_)
{
        int beamNumber = points.size();

        int interpolation_time_duration = 5 * 1000;

        tf::Stamped<tf::Pose> frame_start_pose;
        tf::Stamped<tf::Pose> frame_mid_pose;
        tf::Stamped<tf::Pose> frame_base_pose;
        tf::Stamped<tf::Pose> frame_end_pose;

        double start_time = startTime.toSec() * 1000 * 1000;
        double end_time = endTime.toSec() * 1000 * 1000;
        double time_inc = (end_time - start_time) / beamNumber; 

        int start_index = 0;
        if(!getLaserPose(frame_start_pose, ros::Time(start_time /1000000.0), tf_))
        {
            ROS_WARN("Not Start Pose,Can not Calib");
            return ;
        }

        // 写入frame_end_pose
        if(!getLaserPose(frame_end_pose,ros::Time(end_time / 1000000.0),tf_))
        {
            ROS_WARN("Not End Pose, Can not Calib");
            return ;
        }

        int cnt = 0;
        frame_base_pose = frame_start_pose;
        for(int i = 0; i < beamNumber; i++)
        {
            double mid_time = start_time + time_inc * (i - start_index);
            if(mid_time - start_time > interpolation_time_duration || (i == beamNumber - 1))
            {
                cnt++;
                if(!getLaserPose(frame_mid_pose, ros::Time(mid_time/1000000.0), tf_))
                {
                    ROS_ERROR("Mid %d Pose Error",cnt);
                    return ;
                }

                int interp_count = i - start_index + 1;
                MapBuilder::laserMotionCalibration(frame_base_pose,
                                        frame_start_pose,
                                        frame_mid_pose,  
                                        points,
                                        start_index,
                                        interp_count);

                start_time = mid_time;
                start_index = i;
                frame_start_pose = frame_mid_pose;
            }
        }
}


void MapBuilder::laserMotionCalibration(tf::Stamped<tf::Pose> frame_base_pose,
                                             tf::Stamped<tf::Pose> frame_start_pose,
                                             tf::Stamped<tf::Pose> frame_end_pose,
                                             std::vector<Point> &points,
                                             int startIndex,
                                             int beam_number  
                                            )
{
        double beam_step = 1.0 / (beam_number - 1);
        tf::Quaternion start_angle_q = frame_start_pose.getRotation();
        tf::Quaternion end_angle_q = frame_end_pose.getRotation();
        double start_angle_r = tf::getYaw(start_angle_q);
        double base_angle_r = tf::getYaw(frame_base_pose.getRotation());
        tf::Vector3 start_pos = frame_start_pose.getOrigin();
        start_pos.setZ(0);
        tf::Vector3 end_pos = frame_end_pose.getOrigin();
        end_pos.setZ(0);
        tf::Vector3 base_pos = frame_base_pose.getOrigin();
        base_pos.setZ(0);
        double mid_angle;
        tf::Vector3 mid_pos;
        tf::Vector3 mid_point;
        double lidar_angle, lidar_dist;
        for (int i = 0; i <beam_number; i++)
        {
            if (this->calib)
            {
                mid_angle = tf::getYaw(start_angle_q.slerp(end_angle_q, beam_step * i));
                mid_pos = start_pos.lerp(end_pos, beam_step * i);
            }
            else
            {
                mid_angle = start_angle_r;
                mid_pos = start_pos;
            }
            double tmp_angle;
            if (!isinf(points[startIndex + i].range))
            {
                lidar_dist = points[startIndex + i].range;
                lidar_angle = points[startIndex + i].angle;
                double laser_x, laser_y;
                laser_x = lidar_dist * cos(lidar_angle);
                laser_y = lidar_dist * sin(lidar_angle);
                double odom_x, odom_y;
                odom_x = laser_x * cos(mid_angle) - laser_y * sin(mid_angle) + mid_pos.x();
                odom_y = laser_x * sin(mid_angle) + laser_y * cos(mid_angle) + mid_pos.y();
                // =============================================================================================================
                /*
                 *  base -> odom  [c -s x0;
                 *                       s c y0;
                 *                       0 0 1]
                 *  odom -> base  [c s -x0*c-y0*s;
                 *                       -s c x0*s - y0*c;
                 *                        0 0 1]
                 */
                // =============================================================================================================

                this->occ_map->rayCast(odom_x, odom_y, mid_pos);
            }
            
            else
            {
//                 std::cout<<"NULL"<<std::endl;
//                 lidar_angle = points[startIndex + i].angle;
//                 tmp_angle = mid_angle + lidar_angle;
//                 tmp_angle = tfNormalizeAngle(tmp_angle);
//                 lidar_angle = tfNormalizeAngle(tmp_angle - start_angle_r);
//                 angles[startIndex + i] = lidar_angle;
//                     std::cout<<points[startIndex + i].angle<<std::endl;
//                 this->occ_map.rayCast();
//                 std::cout<<"NULL"<<std::endl;
//                 std::pair<int, int> pose_r =  this->occ_map->getCellIndex(mid_pos[0],mid_pos[1]);
                
            }
        }
    }





