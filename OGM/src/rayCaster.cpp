# include<occupancy_grid_mapping/rayCaster.h>

Caster::Caster()
{

}


void Caster::updateMap(int x, int y, int x_b, int y_b, Eigen::MatrixXd& grid_map){
    
//     std::cout<<grid_map.rows()<<std::endl;
//     std::cout<<grid_map.cols()<<std::endl;
//     std::cout<<"ready to cast"<<std::endl;
//     map->gridMap(x, y);
    int eigen_x = this->size_x/2 - x;
    int eigen_y = this->size_y/2 - y;
    
//     int odom_x = this->size_x/2;
//     int odom_y = this->size_y/2;
    int odom_x = this->size_x/2 - x_b;
    int odom_y = this->size_y/2 - y_b;
    
    
    Bresenham(odom_x, odom_y, eigen_x, eigen_y, this->grid_index_vec);
//     std::cout<<"x:"<<eigen_x<<"y:"<<eigen_y<<std::endl;
//     std::cout<<map->grid_map.rows()<<std::endl;
//     std::cout<<map->grid_map.cols()<<std::endl;
    grid_map(eigen_x, eigen_y) =  grid_map(eigen_x, eigen_y) + log(0.6/0.4);
    for (auto miss : this->grid_index_vec)
    {
        grid_map(miss.first, miss.second) =  grid_map(miss.first, miss.second) + log(0.4/0.6);
    }
    grid_index_vec.clear();
//     std::cout<<"debug!!!!!!!!!!!!!"<<std::endl;

    
}

void Caster::Bresenham(int &odom_x, int &odom_y, int &eigen_x, int &eigen_y, std::vector<std::pair<int,int>> &grid_index_vec)
{
  bool steep = abs(eigen_y - odom_y) > abs(eigen_x - odom_x); 
  if (steep)
  {
    std::swap(odom_x, odom_y);  
    std::swap(eigen_x, eigen_y);
  }
  int flag = 0;
  if (odom_x > eigen_x)
  {
    std::swap(odom_x, eigen_x);
    std::swap(odom_y, eigen_y);
    flag = 1;
  }


  int deltaX = eigen_x - odom_x;
  int deltaY = abs(eigen_y - odom_y);
  int error = 0;
  int ystep;
  int y = odom_y;

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
    if (steep)  
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

    if (2 * error >= deltaX) 
    {
      y += ystep;
      error -= deltaX;
    }


    if (flag==0)
        {if(pointX == eigen_x && pointY == eigen_y) continue;}
    else
        {if(pointX == odom_x && pointY == odom_y) continue;}

    grid_index_vec.push_back(std::pair<int, int>(pointX, pointY));
  }

}
