#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <vector>
using std::vector;

#define PI 3.14159
#define rad2rev 0.1591549
#define sec2min 0.0166667
#define deg2rad 0.0174533
#define Wheel_Base 0.255
#define Wheel_Radius 0.075
#define Gear_Ratio 50


ros::Publisher motor_vel_pub;
ros::Publisher slip_pub;
ros::Publisher tilt_pub;
ros::Subscriber vel_sub;
ros::Subscriber imu_sub;
ros::Subscriber odom_sub;

float linear_vel,angular_vel,VR,VL,odom_omega,imu_omega,slip_ratio,tilt_angle,imu_y;
vector<int> motor_vel(2);

 void velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
 linear_vel=msg->linear.x;
 angular_vel=msg->angular.z;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

  odom_omega = msg->twist.twist.angular.z;

}

 void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
 tilt_angle=(1/deg2rad)*acos(msg->linear_acceleration.z/9.81);
 imu_omega=msg->angular_velocity.z;
 imu_y=msg->linear_acceleration.y;
 
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle n;
  motor_vel_pub =  n.advertise<std_msgs::Int32MultiArray>("cmd_motor", 10);
  slip_pub =  n.advertise<std_msgs::Float64>("slip", 10);
  tilt_pub =  n.advertise<std_msgs::Float64>("tilt", 10);

  vel_sub = n.subscribe("cmd_vel", 1, velCallback);
imu_sub = n.subscribe("imu/data", 10, imuCallback);
odom_sub = n.subscribe("odom", 1, odomCallback);
ros::Time::init();
ros::Rate r(20);
while (ros::ok())
  {
    slip_ratio=abs(odom_omega-imu_omega);
    VR = linear_vel*0.3 + angular_vel * Wheel_Base;
    VL = linear_vel*0.3 - angular_vel  * Wheel_Base;
    // if (slip_ratio>0.5)//single wheel mode
    // {
   //   if(VR*VL<0)
    //  {
    //    if(VR<0){VR=0;VL=VL*2;}
    //    else if(VL<0){VL=0;VR=VR*2;}
    //  }
    // }

  //  if(VR*VL<0)
  //{
  //  if(VR<0)VR=0;
  //  else if(VL<0)VL=0;
  //}
   std_msgs::Int32MultiArray cmd;
   std_msgs::Float64 slip_value,tilt_value;

   motor_vel[1]= (VR * Gear_Ratio * rad2rev / (sec2min * Wheel_Radius));
   motor_vel[0]= (VL * Gear_Ratio * rad2rev / (sec2min * Wheel_Radius));
   cmd.data=motor_vel;

 
  //  if(abs(odom_omega/imu_omega)<1)slip_ratio=odom_omega/imu_omega;
  //  else slip_ratio = imu_omega/odom_omega;
   

  
  slip_value.data=slip_ratio;
  tilt_value.data=tilt_angle;
  slip_pub.publish(slip_value);
  tilt_pub.publish(tilt_value);
  motor_vel_pub.publish(cmd);
  //   ROS_INFO("tilt_angle = %d",tilt_angle);
  // ROS_INFO("slip_ratio = %f",slip_ratio);
   ROS_INFO("odom z axis=%f",odom_omega);
   ROS_INFO("imu_z axis = %f",imu_omega);
  ros::spinOnce();
  r.sleep();
  
  }
  return 0;
}
