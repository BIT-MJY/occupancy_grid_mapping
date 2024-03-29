#include <occupancy_grid_mapping/Map.h>


Map::Map(int size_x, int size_y, double resolution):size_x(size_x),size_y(size_y),resolution(resolution){
//     std::cout<<"11111111111111"<<std::endl;
//     this->size_x = 300;
//     this->size_y = 300;
//     this->resolution = 0.1;
    this->marker_pub = this->n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    this->grid_map = Eigen::MatrixXd::Zero(this->size_x, this->size_y);   
    
    this->caster.size_x = this->size_x;
    this->caster.size_y = this->size_y;
    
    
}


std::pair<int,int> Map::getCellIndex(double x, double y){
    
    int cell_x, cell_y;
    if (x>0){
        cell_x = floor((x + resolution/2)/resolution);
    }
    else{
        cell_x = ceil((x - resolution/2)/resolution);
    }
    if (y>0){
        cell_y = floor((y + resolution/2)/resolution);
    }
    else{
        cell_y = ceil((y - resolution/2)/resolution);
    }
    if (cell_x < this->size_x && cell_x > -this->size_x && cell_y<this->size_y && cell_y>-this->size_y)
    {
        return  std::pair<int,int>(cell_x, cell_y);
    }
    else
    {
        return std::pair<int,int>(this->size_x + 10, this->size_y + 10);
    }
}

void Map::rayCast(double x, double y, tf::Vector3 mid_pose)
{
    
    std::pair<int, int> cell = this->getCellIndex(x, y);
    std::pair<int, int> cell_base = this->getCellIndex(mid_pose.x(),mid_pose.y());
    if (cell.first >  this->size_x || cell.second > this->size_y || cell.first <  -this->size_x || cell.second < -this->size_y)
    {
        std::cout<<"out of range!"<<std::endl;
    }
    else
    {
    //     std::cout<<"hit:"<<cell.first<<", "<<cell.second<<std::endl;
        // visual 
        // showPoint(x, y);
        // Bresenham
        caster.updateMap(cell.first, cell.second, cell_base.first, cell_base.second, this->grid_map);
        count2show++;
        if (count2show == 30000)
        {
            this->showMap();
        }
    
    }
}

void Map::showPoint(double x, double y)
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
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0);  // not to disappear
    marker_pub.publish(marker);
}

void Map::showMap()
{
    Eigen::MatrixXd show_map = Eigen::MatrixXd::Zero(this->size_x, this->size_y); 
    for (int i=0; i<this->grid_map.rows();i++)
    {
        for (int j=0; j<this->grid_map.cols(); j++)
        {
            show_map(i,j) =1 - (  1-(1/(1+exp(this->grid_map(i, j))))   );
        }
    }
    cv::Mat dst;
    cv::Mat dst2;
    cv::eigen2cv(show_map, dst);
    dst2 = normalize(dst);
//     cv::imshow("map", dst2);
//     cv::waitKey(0);
    if (this->calib)
    {
        cv::imwrite("/home/mjy/dev/occupancy_grid_mapping/calib.jpg", dst2*255);
        std::cout<<"save the calibrated map!"<<std::endl;
    }
    else
    {
        cv::imwrite("/home/mjy/dev/occupancy_grid_mapping/notcalib.jpg", dst2*255);
        std::cout<<"save the normal map!"<<std::endl;
    }
}


cv::Mat Map::normalize(cv::Mat srcImage)
{

	cv::Mat resultImage = srcImage.clone();
	int nRows = resultImage.rows;
	int nCols = resultImage.cols;
	if(resultImage.isContinuous())
	{
		nCols  = nCols  * nRows;
		nRows = 1;
	}
	uchar *pDataMat;
	int pixMax = 0, pixMin = 255;
	for(int j = 0; j <nRows; j ++)
	{
		pDataMat = resultImage.ptr<uchar>(j);
		for(int i = 0; i < nCols; i ++)
		{
			if(pDataMat[i] > pixMax)       
				pixMax = pDataMat[i];
			if(pDataMat[i] < pixMin)      
				pixMin = pDataMat[i];
		}
	}
	for(int j = 0; j < nRows; j ++)
	{
		pDataMat = resultImage.ptr<uchar>(j);
		for(int i = 0; i < nCols; i ++)
		{
			pDataMat[i] = (pDataMat[i] - pixMin) * 
			    255 / (pixMax - pixMin);
		}
	}
	return resultImage;
}

    
