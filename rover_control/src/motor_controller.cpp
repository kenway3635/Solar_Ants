#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Imu.h>
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
ros::Subscriber vel_sub;
ros::Subscriber imu_sub;

float linear_vel,angular_vel,VR,VL;
int tilt_angle;
vector<int> motor_vel(2);

 void velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
 linear_vel=msg->linear.x;
 angular_vel=msg->angular.z;
}

 void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
 tilt_angle=(1/deg2rad)*acos(msg->linear_acceleration.z/9.81);
 ROS_INFO("tilt_angle = %d",tilt_angle);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle n;
  motor_vel_pub =  n.advertise<std_msgs::Int32MultiArray>("cmd_motor", 10);

  vel_sub = n.subscribe("cmd_vel", 1, velCallback);
imu_sub = n.subscribe("imu/data", 10, imuCallback);
ros::Time::init();
ros::Rate r(20);
while (ros::ok())
  {
    VR = linear_vel + angular_vel * Wheel_Base;
    VL = linear_vel - angular_vel  * Wheel_Base;
    
    // if(VR*VL<0)
    // {
    //   if(VR<0)VR=0;
    //   else if(VL<0)VL=0;
    // }

   std_msgs::Int32MultiArray cmd;
   motor_vel[1]= (VR * Gear_Ratio * rad2rev / (sec2min * Wheel_Radius));
   motor_vel[0]= (VL * Gear_Ratio * rad2rev / (sec2min * Wheel_Radius));
   cmd.data=motor_vel;

  motor_vel_pub.publish(cmd);
  ros::spinOnce();
  r.sleep();
  
  }
  return 0;
}