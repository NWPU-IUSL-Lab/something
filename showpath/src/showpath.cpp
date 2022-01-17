#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include<fstream>
#include<iostream>
#include<iosfwd>
#include<tf/transform_listener.h>

using namespace std;
ofstream foutC;
nav_msgs::Path path;

float t1;
float t2;

int main (int argc, char **argv)
{

    ros::init(argc, argv, "showpath");
    ros::NodeHandle nh;
    
    foutC.open("./test.txt");
    tf::TransformListener listener;
    ros::Rate rate(20);
    while(ros::ok())
{
        tf::StampedTransform transform;
        try
        {
            listener.waitForTransform("/world", "/body", ros::Time(0), ros::Duration(1)); 
            listener.lookupTransform("/world", "/body", ros::Time(0), transform);
            t1=transform.getRotation().getW();
        if(t1!=t2)           
         {
            foutC << transform.stamp_ << " ";
            float x = transform.getOrigin().getX();
            float y = transform.getOrigin().getY();
            float z = transform.getOrigin().getZ();
            float qx = transform.getRotation().getX();
            float qy = transform.getRotation().getY();
            float qz = transform.getRotation().getZ();
            float qw = transform.getRotation().getW();
            ROS_INFO("%f %f %f %f %f %f %f",x,y,z,qx,qy,qz,qw);
            foutC << x <<" " << y << " " << z << " " << qx << " " << qy << " " << qz << " " << qw << std::endl;
            t2=qw;
}
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        rate.sleep();
    }
    foutC.close();
    return 0;

  }

