//要实现的功能：利用fastplanner规划的轨迹信息与飞控发出的定位信息，形成一个闭环的轨迹跟踪控制器
//利用PID做控制算法
//输出为期望推力以及姿态四元数，输出的话题是：/mavros/setpoint_raw/attitude
#include <ros/ros.h>
#include <eigen3/Eigen/Geometry>//用到Eigen的几何模块时需要包含的头文件,即变换矩阵的定义等
#include <eigen3/Eigen/Dense>//用到一些常用的函数时，需要包含的头文件
//#include <sophus/so3.h>//用到李代数数时需要包含的头文件
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include<algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ActuatorControl.h>

#include "control_uav_exp1/PositionCommand.h"
using namespace std;

void calculte_attitude_send(nav_msgs::Odometry& odom_msg,control_uav_exp1::PositionCommand& cmd_msg,float dt);

mavros_msgs::State current_state;
geometry_msgs::TwistStamped vel_cmd;

nav_msgs::Odometry vision_odom;
nav_msgs::Odometry FCU_odom_msg;
nav_msgs::Odometry FCU_odom;
nav_msgs::Odometry FCU_odom_fast;
nav_msgs::Path path;
control_uav_exp1::PositionCommand fast_cmd;

geometry_msgs::TwistStamped FCU_Vel;
geometry_msgs::PoseStamped FCU_Pose;

ros::Publisher local_vel_pub;
ros::Publisher local_acc_pub;
ros::Publisher setpoint_raw_attitude_pub;
ros::Publisher uav_path_pub;
ros::Publisher FCU_odom_fast_pub;

bool offboard_flag=false;
bool first_flag=false;
float current_time,last_time=0.0,last_time1=0.0,_dt,dt,time1,time2;

//以下为姿态控制所需参数以及声明定义
/************************************************************************************/
//下面是位置跟踪pid参数
//最原始的参数：0.95，2.5，0.09，0.2，0.02，0.04，0.01，0.05
//最近的一组参数：0.95，1.1，0.09，0.2，0.02，0.09，0.01，0.05，不加速度，实机跑的
//仿真效果不错的一组参数：0.95，1.5，0.09，0.2，0.15，0.01，0.2，不加速度，仿真跑的
float Kp_cmd_xy=1.25;//0.95;
float Kp_cmd_z=2.5;//2.5
float Kp_cmd_vxvy=0.4;//0.5;
float Kp_cmd_vz=0.2;//0.2
float Ki_cmd_vxvy=0.04;//0.04
float Ki_cmd_vz=0.04;//0.04
float Kd_cmd_vxvy=0.01;//0.01;
float Kd_cmd_vz=0.05;//0.05

float Kp_pos_xy=0.95;//0.95;
float Kp_pos_z=2.5;//2.5
float Kp_pos_vxvy=0.09;//0.09
float Kp_pos_vz=0.2;//0.2
float Ki_pos_vxvy=0.02;//0.02
float Ki_pos_vz=0.04;//0.04
float Kd_pos_vxvy=0.01;//0.01;
float Kd_pos_vz=0.05;//0.05

float Kp_xy=0.95;//0.95;//0.5还可以，0.3不行，因此最低设置为0.5
float Kp_z=1.5;//2.5//1.5
float Kp_vxvy=0.5;//0.09;//最低值：0.03
float Kp_vz=0.2;//0.2
float Ki_vxvy=0.04;//0.02//最低值：0.01
float Ki_vz=0.08;//0.04//0.15
float Kd_vxvy=0.01;//0.01;//最低值：
float Kd_vz=0.06;//0.08//0.2;//0.25

//最大推力阈值设置
float MPC_THR_MAX=0.9,MPC_THR_MIN=0.1;
//最大VELD限幅
float MPC_VELD_LP=5.0;
//最大倾角设置
float tilt_max=5.0;//与加速度定义重复
//期望速度定义
Eigen::Vector3f vel_setpoint(0.0,0.0,0.0);
//速度误差定义
Eigen::Vector3f vel_error(0.0,0.0,0.0);
//比例与微分速度控制定义
Eigen::Vector3f vel_P_output(0.0,0.0,0.0);
Eigen::Vector3f vel_D_output(0.0,0.0,0.0);
//速度误差导数定义
Eigen::Vector3f error_vel_last(0.0,0.0,0.0);
Eigen::Vector3f error_vel_dot_last(0.0,0.0,0.0);
Eigen::Vector3f vel_error_deriv(0.0,0.0,0.0);
//推力控制积分项定义
Eigen::Vector3f thurst_int(0.0,0.0,0.0);
Eigen::Vector3f thrust_sp(0.0,0.0,0.0);
//计算出来的控制推力定义
//Eigen::Vector3d thrust_sp;
float Hover_throttle=0.4;
float thrust_desired_Z1=0.0,delta_thrust_Z=0.0;
//四旋翼最大速度限制
float MPC_XY_VEL_MAX=0.8;
float MPC_Z_VEL_MAX=0.8;


float sign_function(float data)
{
    if(data>0)
    {
        return 1.0;
    }
    else if(data<0)
    {
        return -1.0;
    }
    else if(data == 0)
    {
        return 0.0;
    }
}

float constrain_function(float data, float Max)
{
    if(abs(data)>Max)
    {
        return (data > 0) ? Max : -Max;
    }
    else
    {
        return data;
    }
}

float constrain_function2(float data, float Min,float Max)
{
    if(data > Max)
    {
        return Max;
    }
    else if (data < Min)
    {
        return Min;
    }else
    {
        return data;
    }
}
//旋转矩阵转欧拉角
Eigen::Vector3f quaternion_to_euler(const Eigen::Quaterniond &q)
{
    float quat[4];
    quat[0] = q.w();
    quat[1] = q.x();
    quat[2] = q.y();
    quat[3] = q.z();

    Eigen::Vector3f ans;
    ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
}

void cal_vel_error_deriv(const Eigen::Vector3f& error_now, Eigen::Vector3f& vel_error_deriv,float dt)
{
    Eigen::Vector3f error_vel_dot_now;
    float delta_time=dt;
    error_vel_dot_now = (error_now - error_vel_last)/delta_time;
    //std::cout<<"误差之差: "<<error_now - error_vel_last<<" dt: "<<delta_time<<std::endl;
	//std::cout<<"计算三轴当前微分速度误差: "<<" X: "<<error_vel_dot_now[0]<<" Y: "<<error_vel_dot_now[1]<<" Z: "<<error_vel_dot_now[2]<<std::endl;
    error_vel_last = error_now;
    float a,b;
    b = 2 * M_PI * MPC_VELD_LP * delta_time;//2*pai*5.0*dt
    a = b / (1 + b);

    vel_error_deriv = a * error_vel_dot_now + (1 - a) * error_vel_dot_last ;

    error_vel_dot_last = vel_error_deriv;
}


void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
    //ROS_INFO("connect");
}
void send_path(nav_msgs::Odometry& odom_msg)
{
    //打印运动轨迹
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";
        
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = odom_msg.pose.pose.position.x;
    this_pose_stamped.pose.position.y = odom_msg.pose.pose.position.y;
    this_pose_stamped.pose.position.z = odom_msg.pose.pose.position.z;

    this_pose_stamped.pose.orientation.x = odom_msg.pose.pose.orientation.x;
    this_pose_stamped.pose.orientation.y = odom_msg.pose.pose.orientation.y;
    this_pose_stamped.pose.orientation.z = odom_msg.pose.pose.orientation.z;
    this_pose_stamped.pose.orientation.w = odom_msg.pose.pose.orientation.w;
    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.header.frame_id = "world";
 
    path.poses.push_back(this_pose_stamped);
if(path.poses.size()>3000)
{
path.poses.clear();
}
    //std::cout<<"pose size : "<<path.poses.size()<<std::endl;
    uav_path_pub.publish(path);
    //std::cout<<"pub odom"<<std::endl;
}
void FCU_odom_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //std::cout<<"GET odom"<<std::endl;
    FCU_odom_msg=*msg;
}

void odomcallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    //ROS_INFO("GET odom");
    vision_odom=*msg;
}

void FCU_pose_odomCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    FCU_odom.pose.pose.position.x=msg->pose.position.x;
    FCU_odom.pose.pose.position.y=msg->pose.position.y;
    FCU_odom.pose.pose.position.z=msg->pose.position.z;

    FCU_odom.pose.pose.orientation.x=msg->pose.orientation.x;
    FCU_odom.pose.pose.orientation.y=msg->pose.orientation.y;
    FCU_odom.pose.pose.orientation.z=msg->pose.orientation.z;
    FCU_odom.pose.pose.orientation.w=msg->pose.orientation.w;
}
void fcu_velocity_odom_callback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    //std::cout<<"GET velocity"<<std::endl;

    FCU_odom.twist.twist.linear.x=msg->twist.linear.x;
    FCU_odom.twist.twist.linear.y=msg->twist.linear.y;
    FCU_odom.twist.twist.linear.z=msg->twist.linear.z;
}


void send_2fastplanner()
{
    FCU_odom.header.stamp = ros::Time::now();
    FCU_odom_fast_pub.publish(FCU_odom);
}


void cmdCallback(const control_uav_exp1::PositionCommand::ConstPtr& msg) 
{  
    offboard_flag=true;
    //ROS_INFO("GET cmd");
    fast_cmd=*msg;  
}


void calculte_attitude_send(nav_msgs::Odometry& odom_msg,control_uav_exp1::PositionCommand& cmd_msg,float dt,bool flag)
{

	if (flag==false)
	{
		Kp_xy=Kp_pos_xy;
		Kp_z=Kp_pos_z;
		Kp_vxvy=Kp_pos_vxvy;
		Kp_vz=Kp_pos_vz;
		Ki_vxvy=Ki_pos_vxvy;
		Ki_vz=Ki_pos_vz;
		Kd_vxvy=Kd_pos_vxvy;
		Kd_vz=Kd_pos_vz;
	}
	else
	{
		Kp_xy=Kp_cmd_xy;
		Kp_z=Kp_cmd_z;
		Kp_vxvy=Kp_cmd_vxvy;
		Kp_vz=Kp_cmd_vz;
		Ki_vxvy=Ki_cmd_vxvy;
		Ki_vz=Ki_cmd_vz;
		Kd_vxvy=Kd_cmd_vxvy;
		Kd_vz=Kd_cmd_vz;
	}
    float yaw_sp=cmd_msg.yaw;
    
    float delta_x = cmd_msg.position.x - odom_msg.pose.pose.position.x;
    float delta_y = cmd_msg.position.y - odom_msg.pose.pose.position.y;
    float delta_z = cmd_msg.position.z - odom_msg.pose.pose.position.z;
	std::cout<<"期望三轴位置: "<<" X: "<<cmd_msg.position.x<<" Y: "<<cmd_msg.position.y<<" Z: "<<cmd_msg.position.z<<std::endl;
    std::cout<<"实际三轴位置: "<<" X: "<<odom_msg.pose.pose.position.x<<" Y: "<<odom_msg.pose.pose.position.y<<" Z: "<<odom_msg.pose.pose.position.z<<std::endl;

    delta_x=(int)(delta_x*1000)/1000.0;
    delta_y=(int)(delta_y*1000)/1000.0;
    delta_z=(int)(delta_z*1000)/1000.0; 

    vel_setpoint[0] =cmd_msg.velocity.x + Kp_xy * delta_x;
    vel_setpoint[1] =cmd_msg.velocity.y + Kp_xy * delta_y;
    vel_setpoint[2] =cmd_msg.velocity.z + Kp_z * delta_z;

    //send_path(odom_msg);
//std::cout<<"kp_z: "<<Kp_z<<std::endl;
    std::cout<<"计算三轴位置误差: "<<" X: "<<delta_x<<" Y: "<<delta_y<<" Z: "<<delta_z<<std::endl;
    
    vel_setpoint[0] = constrain_function(vel_setpoint[0], MPC_XY_VEL_MAX);//0.8
    vel_setpoint[1] = constrain_function(vel_setpoint[1], MPC_XY_VEL_MAX);//0.8
    vel_setpoint[2] = constrain_function(vel_setpoint[2], MPC_Z_VEL_MAX);//0.8

    std::cout<<"轨迹速度: "<<" X: "<<cmd_msg.velocity.x<<" Y: "<<cmd_msg.velocity.y<<" Z: "<<cmd_msg.velocity.z<<std::endl;    
    std::cout<<"期望速度: "<<" X: "<<vel_setpoint[0]<<" Y: "<<vel_setpoint[1]<<" Z: "<<vel_setpoint[2]<<std::endl;
    //缺少一个速度误差计算项
    //
    //vel_error[0]=vel_setpoint[0];
    //vel_error[1]=vel_setpoint[1];  
    vel_error[0]=vel_setpoint[0]-odom_msg.twist.twist.linear.x;
    vel_error[1]=vel_setpoint[1]-odom_msg.twist.twist.linear.y;
    vel_error[2]=vel_setpoint[2]-odom_msg.twist.twist.linear.z;
    
    std::cout<<"11计算三轴速度误差: "<<" X: "<<vel_error[0]<<" Y: "<<vel_error[1]<<" Z: "<<vel_error[2]<<std::endl;
    
    vel_P_output[0] = Kp_vxvy * vel_error[0];
    vel_P_output[1] = Kp_vxvy * vel_error[1];
    vel_P_output[2] = Kp_vz  * vel_error[2];
  
    std::cout<<"计算三轴比例速度: "<<" X: "<<vel_P_output[0]<<" Y: "<<vel_P_output[1]<<" Z: "<<vel_P_output[2]<<std::endl;

    cal_vel_error_deriv(vel_error, vel_error_deriv,dt);

    vel_D_output[0] = Kd_vxvy * vel_error_deriv[0];
    vel_D_output[1] = Kd_vxvy * vel_error_deriv[1];
    vel_D_output[2] = Kd_vz  * vel_error_deriv[2];
    std::cout<<"计算三轴微分速度: "<<" X: "<<vel_D_output[0]<<" Y: "<<vel_D_output[1]<<" Z: "<<vel_D_output[2]<<std::endl;
    
    //计算Z轴的推力
    //需要将期望加速度加上
    float thrust_desired_Z  =/*cmd_msg.acceleration.z +*/ vel_P_output[2] + thurst_int[2] + vel_D_output[2] + Hover_throttle;//Hover_throttle=0.4
    std::cout<<"vel_P_output: "<<vel_P_output[2]<<" thurst_int: "<<thurst_int[2]<<" vel_D_output: "<<vel_D_output[2]<<std::endl;
    std::cout<<"计算Z轴的推力: "<<thrust_desired_Z<<std::endl;
    //积分项抗饱和措施，anti-windup
    bool stop_integral_Z = ( thrust_desired_Z  >= MPC_THR_MAX && vel_error[2] >= 0.0f)||( thrust_desired_Z  <= MPC_THR_MIN && vel_error[2] <= 0.0f);//0.9/0.1
    if (!stop_integral_Z) 
    {
        thurst_int[2] += Ki_vz  * vel_error[2] * dt;
        // limit thrust integral
        thurst_int[2] = min(fabs(thurst_int[2]), MPC_THR_MAX ) * sign_function(thurst_int[2]);
    }
    //对推力限幅
    thrust_sp[2] = constrain_function2( thrust_desired_Z , MPC_THR_MIN, MPC_THR_MAX);//0.9/0.1
    //利用嗯PID控制计算X，Y方向的推力
    float thrust_desired_X;
    float thrust_desired_Y;
    //需要加上期望加速度
    thrust_desired_X  =/*cmd_msg.acceleration.x + */ vel_P_output[0] + thurst_int[0] + vel_D_output[0];
    thrust_desired_Y  =/*cmd_msg.acceleration.y + */ vel_P_output[1] + thurst_int[1] + vel_D_output[1];
    //根据最大倾角计算X，Y方向上的最大推力
    float thrust_max_XY_tilt = fabs(thrust_sp[2]) * tanf(tilt_max/180.0*M_PI);//我觉得应该是cos
    float thrust_max_XY = sqrtf(MPC_THR_MAX * MPC_THR_MAX - thrust_sp[2] * thrust_sp[2]);
    thrust_max_XY = min(thrust_max_XY_tilt, thrust_max_XY);
    //对X，Y轴上的推力赋值
    thrust_sp[0] = thrust_desired_X;
    thrust_sp[1] = thrust_desired_Y;
    //对X，Y轴的推力限幅.按比例对归一化推力进行计算
    if ((thrust_desired_X * thrust_desired_X + thrust_desired_Y * thrust_desired_Y) > thrust_max_XY * thrust_max_XY) 
    {
        float mag = sqrtf((thrust_desired_X * thrust_desired_X + thrust_desired_Y * thrust_desired_Y));
        thrust_sp[0] = thrust_desired_X / mag * thrust_max_XY;
        thrust_sp[1] = thrust_desired_Y / mag * thrust_max_XY;
    }
    std::cout<<"三轴推力: "<<" X: "<<thrust_sp[0]<<" Y: "<<thrust_sp[1]<<" Z: "<<thrust_sp[2]<<std::endl;
    // Use tracking Anti-Windup for XY-direction: during saturation, the integrator is used to unsaturate the output
    // see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
    float arw_gain = 2.f / Kp_vxvy;
    float vel_err_lim_x,vel_err_lim_y;
    vel_err_lim_x = vel_error[0] - (thrust_desired_X - thrust_sp[0]) * arw_gain;
    vel_err_lim_y = vel_error[1] - (thrust_desired_Y - thrust_sp[1]) * arw_gain;

    // 更新积分项
    thurst_int[0] += Ki_vxvy * vel_err_lim_x * dt;
    thurst_int[1] += Ki_vxvy * vel_err_lim_y * dt;
    //将三轴推力转变为Z轴的推力以及期望姿态
    Eigen::Vector3f body_x, body_y, body_z;
    double thr_sp_length = thrust_sp.norm();//求推力的模

 if (thr_sp_length > 0.00001f) 
    {
        body_z = thrust_sp.normalized();//三轴推力的归一化向量
    } 
    else 
    {
        // no thrust, set Z axis to safe value
        body_z = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
    }

    // vector of desired yaw direction in XY plane, rotated by PI/2
    Eigen::Vector3f y_C = Eigen::Vector3f(-sinf(yaw_sp),cosf(yaw_sp),0.0);

    if (fabsf(body_z(2)) > 0.000001f) 
    {
        // desired body_x axis, orthogonal to body_z
        body_x = y_C.cross(body_z);

        // keep nose to front while inverted upside down
        if (body_z(2) < 0.0f) 
        {
            body_x = -body_x;
        }

        body_x.normalize();
   } else 
   {
        // desired thrust is in XY plane, set X downside to construct correct matrix,
        // but yaw component will not be used actually
        body_x = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        body_x(2) = 1.0f;
    }
 
    body_y = body_z.cross(body_x);
    Eigen::Matrix3d R_sp;
    // 求解旋转矩阵，即R_sp=[body_x,body_y,body_z],即表示期望的姿态
    for (int i = 0; i < 3; i++) 
    {
        R_sp(i, 0) = body_x(i);
        R_sp(i, 1) = body_y(i);
        R_sp(i, 2) = body_z(i);
    }
    Eigen::Quaterniond q_sp(R_sp);//将姿态转变为四元数

    mavros_msgs::AttitudeTarget att_setpoint;//发送

    //Mappings: If any of these bits are set, the corresponding input should be ignored:
    //bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude

    att_setpoint.type_mask = 0b00000111;

    att_setpoint.orientation.x = q_sp.x();
    att_setpoint.orientation.y = q_sp.y();
    att_setpoint.orientation.z = q_sp.z();
    att_setpoint.orientation.w = q_sp.w();

    att_setpoint.thrust = thr_sp_length;

    setpoint_raw_attitude_pub.publish(att_setpoint);
    std::cout<<"最终推力："<<thr_sp_length<<std::endl;
    Eigen::Vector3f Euler;
    Euler=quaternion_to_euler(q_sp);
    std::cout<<"姿态控制："<<" 滚转角 "<<Euler[0]*180/M_PI<<" 俯仰角 "<<Euler[1]*180/M_PI<<" 偏航角 "<<Euler[2]*180/M_PI<<std::endl;
        if( current_state.mode != "OFFBOARD" )
    {
        thurst_int<<0.0,0.0,0.0;
        //ROS_INFO("wait Offboard enabled555");
    } 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_uav1");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    ros::Subscriber cmd_sub = nh.subscribe("/planning/pos_cmd", 50, cmdCallback);//100hz

    ros::Subscriber fcu_position_sub = nh.subscribe("mavros/local_position/pose", 50, FCU_pose_odomCallback);//30hz

    ros::Subscriber fcu_velocity_odom_sub = nh.subscribe("/mavros/local_position/velocity_local",50,fcu_velocity_odom_callback);//30hz

    FCU_odom_fast_pub = nh.advertise<nav_msgs::Odometry>("/fcu_odom/odom", 100);

    setpoint_raw_attitude_pub= nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);//80hz

    uav_path_pub = nh.advertise<nav_msgs::Path>("/trajectory1", 1, true);

    //发送频率必须大于2hz
    ros::Rate rate(100.0);//50

    // 等待mavros连接
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        ROS_INFO("NOT OK");
        rate.sleep();
    }

    control_uav_exp1::PositionCommand fast_cmd1;
    fast_cmd1.position.x=0.0;
    fast_cmd1.position.y=0.0;
    fast_cmd1.position.z=0.5;

	fast_cmd1.velocity.x=0.0;
	fast_cmd1.velocity.y=0.0;
	fast_cmd1.velocity.z=0.0;

	fast_cmd1.acceleration.x=0.0;
	fast_cmd1.acceleration.y=0.0;
	fast_cmd1.acceleration.z=0.0;

	fast_cmd1.yaw=0.0;

  //  time1=ros::Time::now().toSec();
  //  last_time=time1;
   //读一些飞控的数据
    for(int i = 100; ros::ok() && i > 0; --i)
{
        //current_time=ros::Time::now().toSec();
        //dt = current_time - last_time;
        //dt = constrain_function2(dt,0.01,0.03);
        //last_time=current_time;

		//calculte_attitude_send(FCU_odom,fast_cmd1,dt);
        ros::spinOnce();
        rate.sleep();
    }
    time1=ros::Time::now().toSec();
    last_time=time1;
    while(ros::ok())
    {
        current_time=ros::Time::now().toSec();
        _dt = current_time - last_time;
        dt = constrain_function2(_dt,0.01,0.03);
        last_time=current_time;

        ros::spinOnce();//检测是否接收到话题
        send_2fastplanner();
    	//等待飞机切换为offboard模式以及解锁飞控
    	/*
    	 if( current_state.mode != "OFFBOARD" )
        {
            ROS_INFO("wait Offboard enabled");
        } 
        else 
        {
            if( !current_state.armed )
            {
            ROS_INFO(" wait Vehicle armed"); 
            }
        }
*/
        if (offboard_flag==false)
        {
        	calculte_attitude_send(FCU_odom,fast_cmd1,dt,false);
        }
        else
        {
            calculte_attitude_send(FCU_odom,fast_cmd,dt,true);
        }
        rate.sleep();
    }
    return 0;
}
