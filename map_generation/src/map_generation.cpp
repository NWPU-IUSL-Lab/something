#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<pcl/io/pcd_io.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <random>

using namespace std;
using namespace Eigen;

double _x_size=50.0, _y_size=70.0, _z_size=5.0;//地图大小；//20,20,5
double w_weigh=1;//墙的宽度
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
double _z_limit, _sensing_range, _resolution=0.1, _sense_rate, _init_x, _init_y;

pcl::PointCloud<pcl::PointXYZ> cloudMap;

void RandomMapGenerate() 
{
  //首先根据地图大小构建墙壁

  Vector3i map1_min_index;
  map1_min_index[0] = ceil(_x_l/_resolution);
  map1_min_index[1] = ceil(_y_l/_resolution);
  map1_min_index[2] = ceil(-1.0/_resolution);

  Vector3i map1_max_index;
  map1_max_index[0] = (_x_h/_resolution);
  map1_max_index[1] = (_y_h/_resolution);
  map1_max_index[2] = (4.0/_resolution);

  Vector3i map2_min_index;
  map2_min_index[0] = ceil((_x_l-1.0)/_resolution);
  map2_min_index[1] = ceil((_y_l-1.0)/_resolution);
  map2_min_index[2] = ceil((-1.0-1.0)/_resolution);

  Vector3i map2_max_index;
  map2_max_index[0] = ((_x_h+1.0)/_resolution);
  map2_max_index[1] = ((_y_h+1.0)/_resolution);
  map2_max_index[2] = ((4.0+1.0)/_resolution);

  pcl::PointXYZ pt_obs;

//cout<<"map2_min_index"<<map2_min_index[0]<<","<<map2_max_index[0]<<endl;
//cout<<"map1_min_index"<<map1_min_index[1]<<","<<map2_max_index[0]<<endl;
//右墙
  for(int i=map2_min_index[0];i<=map2_max_index[0];i++)
  {
    for(int j=map1_min_index[1];j>=map2_min_index[1];j--)
    {
      for(int l=map1_min_index[2];l<=map1_max_index[2];l++)
      {
        pt_obs.x=i*_resolution;
        pt_obs.y=j*_resolution;
        pt_obs.z=l*_resolution;
        cloudMap.points.push_back(pt_obs);
      }
    }
  }
//左墙
  for(int i=map1_min_index[0];i<=map2_max_index[0];i++)
  {
    for(int j=map2_max_index[1];j<=map2_max_index[1];j++)
    {
      for(int l=map1_min_index[2];l<=map1_max_index[2];l++)
      {
        pt_obs.x=i*_resolution;
        pt_obs.y=j*_resolution;
        pt_obs.z=l*_resolution;

        cloudMap.points.push_back(pt_obs);
      }
    }
  }
//上墙

 for(int i=map1_max_index[0];i<=map2_max_index[0];i++)
  {
    for(int j=map2_min_index[1];j<=map2_max_index[1];j++)
    {
      for(int l=map1_min_index[2];l<=map1_max_index[2];l++)
      {
        pt_obs.x=i*_resolution;
        pt_obs.y=j*_resolution;
        pt_obs.z=l*_resolution;

        cloudMap.points.push_back(pt_obs);
      }
    }
  }

//下墙
 for(int i=map1_min_index[0];i>=map2_min_index[0];i--)
  {
    for(int j=map2_min_index[1];j<=map2_max_index[1];j++)
    {
      for(int l=map1_min_index[2];l<=map1_max_index[2];l++)
      {
        pt_obs.x=i*_resolution;
        pt_obs.y=j*_resolution;
        pt_obs.z=l*_resolution;

        cloudMap.points.push_back(pt_obs);
      }
    }
  }

  //地图2楼道无障碍物，只有墙
//第一堵墙：x:5-6,y:15-35,z:-1-3.0

  map1_min_index[0] = ceil(5.0/_resolution);
  map1_min_index[1] = ceil(15.0/_resolution);
  map1_min_index[2] = ceil(-1.0/_resolution);

  map1_max_index[0] = (6.0/_resolution);
  map1_max_index[1] = (36.0/_resolution);
  map1_max_index[2] = (4.0/_resolution);

   for(int i=map1_min_index[0];i<=map1_max_index[0];i++)
  {
    for(int j=map1_min_index[1];j<=map1_max_index[1];j++)
    {
      for(int l=map1_min_index[2];l<=map1_max_index[2];l++)
      {
        pt_obs.x=i*_resolution;
        pt_obs.y=j*_resolution;
        pt_obs.z=l*_resolution;
        cloudMap.points.push_back(pt_obs);
      }
    }
  }
//第二堵墙：x:5-25,y:9-10,z:-1-4
  map1_min_index[0] = ceil(5.0/_resolution);
  map1_min_index[1] = ceil(9.0/_resolution);
  map1_min_index[2] = ceil(-1.0/_resolution);

  map1_max_index[0] = (25.0/_resolution);
  map1_max_index[1] = (10.0/_resolution);
  map1_max_index[2] = (4.0/_resolution);

   for(int i=map1_min_index[0];i<=map1_max_index[0];i++)
  {
    for(int j=map1_min_index[1];j<=map1_max_index[1];j++)
    {
      for(int l=map1_min_index[2];l<=map1_max_index[2];l++)
      {
        pt_obs.x=i*_resolution;
        pt_obs.y=j*_resolution;
        pt_obs.z=l*_resolution;
        cloudMap.points.push_back(pt_obs);
      }
    }
  }

  //第三堵墙：x:5-6,y:-5-9,z:-1-4
  map1_min_index[0] = ceil(5.0/_resolution);
  map1_min_index[1] = ceil(-5.0/_resolution);
  map1_min_index[2] = ceil(-1.0/_resolution);

  map1_max_index[0] = (6.0/_resolution);
  map1_max_index[1] = (9.0/_resolution);
  map1_max_index[2] = (4.0/_resolution);

   for(int i=map1_min_index[0];i<=map1_max_index[0];i++)
  {
    for(int j=map1_min_index[1];j<=map1_max_index[1];j++)
    {
      for(int l=map1_min_index[2];l<=map1_max_index[2];l++)
      {
        pt_obs.x=i*_resolution;
        pt_obs.y=j*_resolution;
        pt_obs.z=l*_resolution;
        cloudMap.points.push_back(pt_obs);
      }
    }
  }

    //第四堵墙：x:5-25,y:-11--10,z:-1-4
  map1_min_index[0] = ceil(5.0/_resolution);
  map1_min_index[1] = ceil(-11.0/_resolution);
  map1_min_index[2] = ceil(-1.0/_resolution);

  map1_max_index[0] = (25.0/_resolution);
  map1_max_index[1] = (-10.0/_resolution);
  map1_max_index[2] = (4.0/_resolution);

   for(int i=map1_min_index[0];i<=map1_max_index[0];i++)
  {
    for(int j=map1_min_index[1];j<=map1_max_index[1];j++)
    {
      for(int l=map1_min_index[2];l<=map1_max_index[2];l++)
      {
        pt_obs.x=i*_resolution;
        pt_obs.y=j*_resolution;
        pt_obs.z=l*_resolution;
        cloudMap.points.push_back(pt_obs);
      }
    }
  }

  //第五堵墙：x:5-6,y:-30--11,z:-1-4
  map1_min_index[0] = ceil(5.0/_resolution);
  map1_min_index[1] = ceil(-30.0/_resolution);
  map1_min_index[2] = ceil(-1.0/_resolution);

  map1_max_index[0] = (6.0/_resolution);
  map1_max_index[1] = (-11.0/_resolution);
  map1_max_index[2] = (4.0/_resolution);

   for(int i=map1_min_index[0];i<=map1_max_index[0];i++)
  {
    for(int j=map1_min_index[1];j<=map1_max_index[1];j++)
    {
      for(int l=map1_min_index[2];l<=map1_max_index[2];l++)
      {
        pt_obs.x=i*_resolution;
        pt_obs.y=j*_resolution;
        pt_obs.z=l*_resolution;
        cloudMap.points.push_back(pt_obs);
      }
    }
  }
    //第六堵墙：x:-6--5,y:15-36,z:-1-4
  map1_min_index[0] = ceil(-6.0/_resolution);
  map1_min_index[1] = ceil(15.0/_resolution);
  map1_min_index[2] = ceil(-1.0/_resolution);

  map1_max_index[0] = (-5.0/_resolution);
  map1_max_index[1] = (36.0/_resolution);
  map1_max_index[2] = (4.0/_resolution);

   for(int i=map1_min_index[0];i<=map1_max_index[0];i++)
  {
    for(int j=map1_min_index[1];j<=map1_max_index[1];j++)
    {
      for(int l=map1_min_index[2];l<=map1_max_index[2];l++)
      {
        pt_obs.x=i*_resolution;
        pt_obs.y=j*_resolution;
        pt_obs.z=l*_resolution;
        cloudMap.points.push_back(pt_obs);
      }
    }
  }
      //第七堵墙：x:-25--11,y:15-16,z:-1-4
  map1_min_index[0] = ceil(-25.0/_resolution);
  map1_min_index[1] = ceil(15.0/_resolution);
  map1_min_index[2] = ceil(-1.0/_resolution);

  map1_max_index[0] = (-11.0/_resolution);
  map1_max_index[1] = (16.0/_resolution);
  map1_max_index[2] = (4.0/_resolution);

   for(int i=map1_min_index[0];i<=map1_max_index[0];i++)
  {
    for(int j=map1_min_index[1];j<=map1_max_index[1];j++)
    {
      for(int l=map1_min_index[2];l<=map1_max_index[2];l++)
      {
        pt_obs.x=i*_resolution;
        pt_obs.y=j*_resolution;
        pt_obs.z=l*_resolution;
        cloudMap.points.push_back(pt_obs);
      }
    }
  }
   //第八堵墙：x:-5--6,y:-35--15,z:-1-4
  map1_min_index[0] = ceil(-6.0/_resolution);
  map1_min_index[1] = ceil(-35.0/_resolution);
  map1_min_index[2] = ceil(-1.0/_resolution);

  map1_max_index[0] = (-5.0/_resolution);
  map1_max_index[1] = (-15.0/_resolution);
  map1_max_index[2] = (4.0/_resolution);

   for(int i=map1_min_index[0];i<=map1_max_index[0];i++)
  {
    for(int j=map1_min_index[1];j<=map1_max_index[1];j++)
    {
      for(int l=map1_min_index[2];l<=map1_max_index[2];l++)
      {
        pt_obs.x=i*_resolution;
        pt_obs.y=j*_resolution;
        pt_obs.z=l*_resolution;
        cloudMap.points.push_back(pt_obs);
      }
    }
  }
   //第九堵墙：x:-25--11,y:-15--16,z:-1-4
  map1_min_index[0] = ceil(-25.0/_resolution);
  map1_min_index[1] = ceil(-16.0/_resolution);
  map1_min_index[2] = ceil(-1.0/_resolution);

  map1_max_index[0] = (-11.0/_resolution);
  map1_max_index[1] = (-15.0/_resolution);
  map1_max_index[2] = (4.0/_resolution);

   for(int i=map1_min_index[0];i<=map1_max_index[0];i++)
  {
    for(int j=map1_min_index[1];j<=map1_max_index[1];j++)
    {
      for(int l=map1_min_index[2];l<=map1_max_index[2];l++)
      {
        pt_obs.x=i*_resolution;
        pt_obs.y=j*_resolution;
        pt_obs.z=l*_resolution;
        cloudMap.points.push_back(pt_obs);
      }
    }
  }
  /*
	//障碍物，给定X,Y坐标，绘制边长为2的长方体
	//vector<Vector3d> obs_vec={5,5,0;5,-1,0;5,-6,0;-3,2,0;-3,-2,0;-6,0,0};
	vector<Vector3d> obs_vec;
	//vector<double> obs;
	Vector3d obs;
	obs<<5,5,0;
	obs_vec.push_back(obs);
	obs<<5,-1,0;
	obs_vec.push_back(obs);
	obs<<5,-6,0;
	obs_vec.push_back(obs);
	obs<<-3,2,0;
	obs_vec.push_back(obs);
	obs<<-3,-2,0;
	obs_vec.push_back(obs);
	obs<<-6,0,0;
	obs_vec.push_back(obs);

	Vector3i boud_min,boud_max;

	for(int k=0;k<obs_vec.size();k++)
	{
		boud_min[0]=ceil((obs_vec[k][0]-1.0)/0.1);
		boud_min[1]=ceil((obs_vec[k][1]-1.0)/0.1);
		boud_min[2]=ceil((obs_vec[k][2]-1.0)/0.1);

		boud_max[0]=ceil((obs_vec[k][0]+1.0)/0.1);
		boud_max[1]=ceil((obs_vec[k][1]+1.0)/0.1);
		boud_max[2]=ceil((obs_vec[k][2]+2.0)/0.1);

		for(int i=boud_min[0];i<=boud_max[0];i++)
		{
	   		for(int j=boud_min[1];j<=boud_max[1];j++)
	   		{
	    		for(int l=boud_min[2];l<=boud_max[2];l++)
	     		{	
		    		pt_obs.x=i*_resolution;
        			pt_obs.y=j*_resolution;
        			pt_obs.z=l*_resolution;
        			cloudMap.points.push_back(pt_obs);
	     		}
	   		}
		}
	}
*/

  	cloudMap.width = cloudMap.points.size();

  	cloudMap.height = 1;
  	cloudMap.is_dense = true;

  	ROS_WARN("Finished generate random map ");

}


int main(int argc, char** argv) 
{
  ros::init(argc, argv, "random_map_sensing");/*初始化节点*/
  ros::NodeHandle n("~");

  _x_l = -_x_size / 2.0;/*将_x_l的值赋为地图X的范围的负一半，即-25*/
  _x_h = +_x_size / 2.0;/*将_x_h的值赋为地图X的范围的一半，即25*/

  _y_l = -_y_size / 2.0;/*将_y_l的值赋为地图Y的范围的负一半，即-25*/
  _y_h = +_y_size / 2.0;/*将_y_l的值赋为地图X的范围的一半，即25*/

  ros::Duration(0.5).sleep();/*休息0.5s*/

  RandomMapGenerate();/*产生随机地图*/
  pcl::io::savePCDFileASCII<pcl::PointXYZ>("/home/dtpmh/work_space_all/map_generation/src/big_map.pcd", cloudMap);

}
