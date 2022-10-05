#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64.h>
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


float linear_vel,angular_vel,VR,VL;

vector<int> motor_vel(2);

 void velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
 linear_vel=msg->linear.x;
 angular_vel=msg->angular.z;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle n;
  motor_vel_pub =  n.advertise<std_msgs::Int32MultiArray>("cmd_motor", 1);

  vel_sub = n.subscribe("cmd_vel", 1, velCallback);

  ros::Time::init();
  ros::Rate r(30);

  while (ros::ok())
  {
    VR = linear_vel + angular_vel * Wheel_Base;
    VL = linear_vel - angular_vel  * Wheel_Base;
    // if (slip_ratio>0.5)//single wheel mode
    // {
   //   if(VR*VL<0)
    //  {
    //    if(VR<0){VR=0;VL=VL*2;}
    //    else if(VL<0){VL=0;VR=VR*2;}
    //  }
    // }

    // if(VR*VL<0)
    // {
    // if(VR<0)
    //   {
    //     VR=0.5*VR;
    //   }
    // else if(VL<0)
    //   {
    //     VL=0.5*VL;
    //   }
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
