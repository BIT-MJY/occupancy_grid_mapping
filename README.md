# occupancy_grid_mapping
Here is the method to build a grid map and get rid of the laser distortion

# 激光雷达去除畸变以及二维栅格地图的创建

## 20201108 
  提供了一个简短的bag包进行测试。

### 操作步骤：

```cpp
mkdir src
cd src
git clone https://github.com/BIT-MJY/occupancy_grid_mapping.git 
cd ..
catkin_make
source devel/setup.bash
roslaunch occupancy_grid_mapping ogm.launch
rosbag play --clock 2020-10-25-19-34-25.bag
```

### 参数修改
ogm.launch中

* ```<arg name = "map_size_x" default = "500"/>  ```  x方向栅格数目
* ``` <arg name = "map_size_y" default = "500"/> ```  y方向栅格数目
* ``` <arg name = "resolution" default = "0.1"/> ```  分辨率，表示一格为几米
* ```<arg name = "calibration" default = "true"/> ``` 是否使用激光雷达去除畸变
