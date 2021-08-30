#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <eigen3/Eigen/Geometry>//用到Eigen的几何模块时需要包含的头文件,即变换矩阵的定义等
#include <eigen3/Eigen/Dense>//用到一些常用的函数时，需要包含的头文件
#include <eigen3/Eigen/Eigen>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

using namespace Eigen;

mavros_msgs::State current_state;

geometry_msgs::PoseStamped vision,FCU,vision_fast;
Eigen::Vector3d pos_drone_vio;
Eigen::Quaterniond q_fast_vio;
Eigen::Quaterniond q2,q_drone_vio;
ros::Publisher vision_odom_pub,vision_fast_odom_pub;
ros::Publisher uav_path_pub;
nav_msgs::Path path;
float Vx,Vy,Vz;
bool ready_flag=false;
static tf::Quaternion quat_cam, quat_cam_to_body_x, quat_cam_to_body_y, quat_cam_to_body_z, quat_rot_z, quat_body;

Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q)
{
    float quat[4];
    quat[0] = q.w();
    quat[1] = q.x();
    quat[2] = q.y();
    quat[3] = q.z();

    Eigen::Vector3d ans;
    ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
    ROS_INFO("connect");
}
void send_path(const geometry_msgs::PoseStamped odom_msg)
{
    //打印运动轨迹
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";
        
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = odom_msg.pose.position.x;
    this_pose_stamped.pose.position.y = odom_msg.pose.position.y;
    this_pose_stamped.pose.position.z = odom_msg.pose.position.z;

    this_pose_stamped.pose.orientation.x = odom_msg.pose.orientation.x;
    this_pose_stamped.pose.orientation.y = odom_msg.pose.orientation.y;
    this_pose_stamped.pose.orientation.z = odom_msg.pose.orientation.z;
    this_pose_stamped.pose.orientation.w = odom_msg.pose.orientation.w;
    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.header.frame_id = "world";
 if(path.poses.size()>2000)
path.poses.clear();
    path.poses.push_back(this_pose_stamped);
    //std::cout<<"pose size : "<<path.poses.size()<<std::endl;
    uav_path_pub.publish(path);
    //std::cout<<"pub odom"<<std::endl;
}

void FCU_odomCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

	FCU.pose.position.x=msg->pose.position.x;
	FCU.pose.position.y=msg->pose.position.y;
	FCU.pose.position.z=msg->pose.position.z;

	FCU.pose.orientation.x=msg->pose.orientation.x;
	FCU.pose.orientation.y=msg->pose.orientation.y;
	FCU.pose.orientation.z=msg->pose.orientation.z;
	FCU.pose.orientation.w=msg->pose.orientation.w;
}

void vision_odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {


	pos_drone_vio[0]= msg->pose.pose.position.x;
	pos_drone_vio[1]= msg->pose.pose.position.y;
	pos_drone_vio[2]= msg->pose.pose.position.z;
//相机朝向x轴

	vision.pose.position.x = msg->pose.pose.position.x;
	vision.pose.position.y = msg->pose.pose.position.y;
	vision.pose.position.z = msg->pose.pose.position.z;



	vision.pose.orientation.x = msg->pose.pose.orientation.x;
	vision.pose.orientation.y = msg->pose.pose.orientation.y;
	vision.pose.orientation.z = msg->pose.pose.orientation.z;
	vision.pose.orientation.w = msg->pose.pose.orientation.w;

	q_drone_vio.x() = msg->pose.pose.orientation.x;
	q_drone_vio.y() = msg->pose.pose.orientation.y;
	q_drone_vio.z() = msg->pose.pose.orientation.z;
	q_drone_vio.w() = msg->pose.pose.orientation.w;

	//下面是当相机朝向Y轴时，坐标系的旋转关系
/*
	Eigen::AngleAxisd rollAngle(M_PI_2, Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd yawAngle(M_PI_2, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd gammerAngle(-M_PI_2, Eigen::Vector3d::UnitZ());
*/

	Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd pitchAngle(M_PI_2, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd yawAngle(-M_PI_2, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd gammerAngle(0, Eigen::Vector3d::UnitZ());

	q_fast_vio = gammerAngle*q_drone_vio *rollAngle * pitchAngle * yawAngle;
	q_fast_vio.normalize();
}
void send_2fcu()
{
//相机朝向Y轴
	vision.header.stamp = ros::Time::now();
	vision_odom_pub.publish(vision);//给飞控发送的ODOM信息不需要改变，直接发送即可

       send_path(FCU);

    Eigen::Quaterniond q1(vision.pose.orientation.w,vision.pose.orientation.x,vision.pose.orientation.y,vision.pose.orientation.z);//定义旋转四元数
    Eigen::Vector3d eulerAngle1=quaternion_to_euler(q1);

    ROS_INFO("VISION");
    ROS_INFO("x:%.2f ,y:%.2f ,z:%.2f ,Qx:%.2f ,Qy:%.2f ,Qz:%.2f,Qw:%.2f",vision.pose.position.x,vision.pose.position.y,vision.pose.position.z,
        vision.pose.orientation.x,vision.pose.orientation.y,vision.pose.orientation.z,vision.pose.orientation.w);

    ROS_INFO("V_R:%.2f du ,V_P:%.2f du ,V_Y:%.2f du",eulerAngle1(0)*180/M_PI,eulerAngle1(1)*180/M_PI,eulerAngle1(2)*180/M_PI);

    Eigen::Quaterniond q(FCU.pose.orientation.w,FCU.pose.orientation.x,FCU.pose.orientation.y,FCU.pose.orientation.z);//定义旋转四元数
    Eigen::Vector3d eulerAngle=quaternion_to_euler(q);

    ROS_INFO("FCU");
    ROS_INFO("x:%.2f ,y:%.2f ,z:%.2f ,Qx:%.2f ,Qy:%.2f ,Qz:%.2f,Qw:%.2f",FCU.pose.position.x,FCU.pose.position.y,FCU.pose.position.z,
    FCU.pose.orientation.x,FCU.pose.orientation.y,FCU.pose.orientation.z,FCU.pose.orientation.w);

    ROS_INFO("fcu_R:%.2f du ,fcu_P:%.2f du ,fcu_Y:%.2f du",eulerAngle(0)*180/M_PI,eulerAngle(1)*180/M_PI,eulerAngle(2)*180/M_PI);
}
void send_2fastplanner()
{
	vision_fast.pose.position.x=pos_drone_vio[0];
	vision_fast.pose.position.y=pos_drone_vio[1];
	vision_fast.pose.position.z=pos_drone_vio[2];

	vision_fast.pose.orientation.x=q_fast_vio.x();
	vision_fast.pose.orientation.y=q_fast_vio.y();
	vision_fast.pose.orientation.z=q_fast_vio.z();
	vision_fast.pose.orientation.w=q_fast_vio.w();

	vision_fast.header.stamp = ros::Time::now();
    vision_fast_odom_pub.publish(vision_fast);//用来在FASTPLANNER中建图的odom信息，需要转一下坐标系才能正确建图
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_vodom");
    ros::NodeHandle n;

    ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Subscriber fcu_position_sub = n.subscribe("mavros/local_position/pose", 1000, FCU_odomCallback);
            
    //ros::Subscriber vision_sub = n.subscribe("/vins_estimator/imu_propagate", 1000, vision_odomCallback);

    ros::Subscriber vision_sub = n.subscribe("/vins_estimator/odometry", 10, vision_odomCallback, ros::TransportHints().tcpNoDelay());

    vision_odom_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 100);
    
    vision_fast_odom_pub = n.advertise<geometry_msgs::PoseStamped>("/pcl_render_node/camera_pose", 100);
    
    uav_path_pub = n.advertise<nav_msgs::Path>("/trajectory1", 1, true);
    ros::Rate rate(100);//50-100hz即可

    while(ros::ok())
    {
        ros::spinOnce();
//std::cout<<"im in"<<std::endl;
	send_2fcu();
	send_2fastplanner();
        rate.sleep();
    }
    return 0;
}
