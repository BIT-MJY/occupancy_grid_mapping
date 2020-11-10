# include<occupancy_grid_mapping/rayCaster.h>

Caster::Caster()
{

}


void Caster::updateMap(int x, int y, int x_b, int y_b, Eigen::MatrixXd& grid_map){
    
//     std::cout<<grid_map.rows()<<std::endl;
//     std::cout<<grid_map.cols()<<std::endl;
//     std::cout<<"ready to cast"<<std::endl;
//     map->gridMap(x, y);
    // 栅格地图更新
    // odom为array正中间的话，x表示行，y表示列
    // odom是前为x，左为y
    int eigen_x = this->size_x/2 - x;
    int eigen_y = this->size_y/2 - y;
    
//     int odom_x = this->size_x/2;
//     int odom_y = this->size_y/2;
    int odom_x = this->size_x/2 - x_b;
    int odom_y = this->size_y/2 - y_b;
    
    
    // 划线算法
//     std::cout<<"777777777777777777777"<<std::endl;
    Bresenham(odom_x, odom_y, eigen_x, eigen_y, this->grid_index_vec);
    // 更新hit
//     std::cout<<"88888888888888888888"<<std::endl;
//     std::cout<<"x:"<<eigen_x<<"y:"<<eigen_y<<std::endl;
//     std::cout<<map->grid_map.rows()<<std::endl;
//     std::cout<<map->grid_map.cols()<<std::endl;
    grid_map(eigen_x, eigen_y) =  grid_map(eigen_x, eigen_y) + log(0.6/0.4);
//     std::cout<<"99999999999999999999"<<std::endl;
    // 遍历存下的点，更新eigen：：matrix值
    for (auto miss : this->grid_index_vec)
    {
        grid_map(miss.first, miss.second) =  grid_map(miss.first, miss.second) + log(0.4/0.6);
    }
//     std::cout<<"aaaaaaaaaaaaaaaaaaaaaaaaa"<<std::endl;
    grid_index_vec.clear();
//     std::cout<<"debug!!!!!!!!!!!!!"<<std::endl;

    
}

void Caster::Bresenham(int &odom_x, int &odom_y, int &eigen_x, int &eigen_y, std::vector<std::pair<int,int>> &grid_index_vec)
{
  bool steep = abs(eigen_y - odom_y) > abs(eigen_x - odom_x);  // 斜率的绝对值是否在0-1内
  // 如果超过1，则交换xy，因为Bresenham画线算法定义于斜率0-1中
  if (steep)
  {
    std::swap(odom_x, odom_y);  
    std::swap(eigen_x, eigen_y);
  }
  // 保证x0小于x1，也是为了迁就算法的使用条件——这里可以理解为从终点向起点画线，是一样的
  // 但是要打个标记
  int flag = 0;
  if (odom_x > eigen_x)
  {
    std::swap(odom_x, eigen_x);
    std::swap(odom_y, eigen_y);
    flag = 1;
  }

  // 以上工作均是为了约束问题条件

  int deltaX = eigen_x - odom_x;
  int deltaY = abs(eigen_y - odom_y);
  int error = 0;
  int ystep;
  int y = odom_y;

  // 决定y的扫描是递增还是递减
  if (odom_y < eigen_y)
  {
    ystep = 1;
  }
  else
  {
    ystep = -1;
  }

  int pointX;
  int pointY;
  for (int x = odom_x; x <= eigen_x; x++)
  {
    if (steep)  // 斜率绝对值大于1的情况，之前交换了xy坐标，因此在这里要交换回来
    {
      pointX = y;
      pointY = x;
    }
    else
    {
      pointX = x;
      pointY = y;
    }

    error += deltaY;

    if (2 * error >= deltaX)  // 记得，正在计算的Pk，是要决定下一个点画在哪里
    {
      y += ystep;
      error -= deltaX;
    }

    //不包含最后一个点．
    // 这里貌似有bug？x0>x1的情况将起点终点交换
    // 那么这种情况应该不考虑起点才对
    // 所以设置一个flag
    if (flag==0)
        {if(pointX == eigen_x && pointY == eigen_y) continue;}
    else
        {if(pointX == odom_x && pointY == odom_y) continue;}

    //保存所有的点
    grid_index_vec.push_back(std::pair<int, int>(pointX, pointY));
  }

}
