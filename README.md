# occupancy_grid_mapping
Here is the method to build a grid map and get rid of the laser distortion

# Distortion Removing and 2D grid map building

## 20201108 
  A brief ROS bag for test.

### Steps：

#### （1）Standard Runing
```bash
mkdir src
cd src
git clone https://github.com/BIT-MJY/occupancy_grid_mapping.git 
cd ..
catkin_make
source devel/setup.bash
roslaunch occupancy_grid_mapping ogm.launch bag_filename:=/home/mjy/dev/occupancy_grid_mapping/2020-10-25-19-34-25.bag
```
#### （2）Testing
```bash
source devel/setup.bash
roslaunch occupancy_grid_mapping test_ogm.launch bag_filename:=/home/mjy/dev/occupancy_grid_mapping/2020-10-25-19-34-25.bag
```


### Parameters
ogm.launch

* ```<arg name = "map_size_x" default = "500"/>  ```  x方向栅格数目
* ``` <arg name = "map_size_y" default = "500"/> ```  y方向栅格数目
* ``` <arg name = "resolution" default = "0.1"/> ```  分辨率，表示一格为几米
* ```<arg name = "calibration" default = "true"/> ``` 是否使用激光雷达去除畸变

![](https://github.com/BIT-MJY/occupancy_grid_mapping/blob/master/OGM/img/calib.jpg)
