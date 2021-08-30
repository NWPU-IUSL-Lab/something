#include <ros/ros.h>//ros必须的头文件
#include <cmath>

#include <Eigen/Geometry>//用到Eigen的几何模块时需要包含的头文件,即变换矩阵的定义等
#include <Eigen/Dense>//用到一些常用的函数时，需要包含的头文件
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

//pcl
#include <sensor_msgs/PointCloud2.h>  //用到ROS中的点云信息的时候需要包含的头文件
#include <pcl_conversions/pcl_conversions.h>  //用到pcl与ROS点云转换的时候需要包含的头文件
#include <pcl/common/transforms.h>//坐标系变换的时候需要用到的头文件
#include <pcl/point_types.h> //用到定义pcl点的类型时需要包含的头文件
#include <pcl/filters/passthrough.h>//用到直通点云滤波时需要包含的头文件
#include <pcl/filters/statistical_outlier_removal.h>//用到高斯点云滤波的时候需要包含的头文件
#include <pcl/filters/voxel_grid.h>//用到体素滤波时需要包含的头文件
#include <pcl/point_cloud.h>//定义pcl中点云类型的时候需要包含的头文件

#include <thread>//用到线程函数时需要包含的头文件
#include <mutex>//用到线程函数以及互斥量，共同处理共享信息时需要包含的头文件


using namespace std;
using namespace Eigen;

sensor_msgs::PointCloud2 local_map;//用来发布转换到世界坐标系下的点云结果
sensor_msgs::PointCloud2 trans_clouds;//用来接受相机发布的点云结果

/****************************定义用到的共享点云信息*******************************************************************************/
pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_PCL(new pcl::PointCloud<pcl::PointXYZ>);//定义PCL类型的点云

void mapCallback(const sensor_msgs::PointCloud2& map_pointclouds)
{
    //cout<<"get cloud"<<endl;
    trans_clouds=map_pointclouds;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "make_LocalMap");
    ros::NodeHandle n;   

    ros::Subscriber map_sub = n.subscribe("/sdf_map/occupancy_inflate", 1, mapCallback);
Eigen::Vector3f vel_error(1.0,0.0,0.0);
    //ros::Rate rate(10);//50-100hz即可
    while(ros::ok())
    {
        ros::spinOnce();
std::cout<<"坡度： "<<45<<"起点： "<<vel_error[0]<<","<<vel_error[1]<<","<<vel_error[2]<<std::endl;
    }
    pcl::fromROSMsg(trans_clouds, *map_cloud_PCL);//将接收到的ROS类型的点云信息转换成PCL中的类型，以便调用PCL库进行处理
    pcl::io::savePCDFile<pcl::PointXYZ>("world_map.pcd", *map_cloud_PCL);

    return 0;
}
