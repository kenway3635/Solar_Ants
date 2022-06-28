#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class joy_control
{
public:
  joy_control();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_, mode_pub,elevator_arrived_pub,floor_pub,click_pub,brush_pub;
  ros::Subscriber joy_sub_;

};


joy_control::joy_control():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  mode_pub = nh_.advertise<std_msgs::Int32>("/Mode", 1,true);
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  elevator_arrived_pub = nh_.advertise<std_msgs::Int32>("/next_goal", 1,true);
  click_pub=nh_.advertise<std_msgs::Bool>("click",1);
  brush_pub=nh_.advertise<std_msgs::Bool>("brush_switch",1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &joy_control::joyCallback, this);

}

void joy_control::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  std_msgs::Int32 mode,go;
  std_msgs::String floor;
  std_msgs::Bool button;
  std_msgs::Bool brush;
  if (joy->buttons[4])
  {
    twist.angular.z = a_scale_*joy->axes[angular_];
    twist.linear.x = l_scale_*joy->axes[linear_];
  }
  else if(joy->buttons[5]) 
  {
    twist.angular.z = 0;
    twist.linear.x= 0;
  }
  
  else
  {
    twist.angular.z =a_scale_*joy->axes[angular_];
    twist.linear.x = l_scale_*joy->axes[linear_];
  }
  vel_pub_.publish(twist);

  if (abs(joy->axes[7])==1)
  {
    if (joy->axes[7]>0)
    {
      brush.data=true;
    }
    else if (joy->axes[7]<0)
    {
      brush.data=false;
    }
    brush_pub.publish(brush);
  }

  if (joy->buttons[1])
  {
    mode.data = 99; //EMERGENCY STOP
    button.data=true;
    mode_pub.publish(mode);
    click_pub.publish(button);
  }
  else if (joy->buttons[0])
  {
    mode.data = 0; //NAVIGATION Mode
    button.data=true;
    mode_pub.publish(mode);
    click_pub.publish(button);
  }
  else if (joy->buttons[2])
  {
    mode.data = 1; //REMOTE Mode
    button.data=true;
    mode_pub.publish(mode);
    click_pub.publish(button);
  }
  else if (joy->buttons[3])
  {
    mode.data = 2; //FOLLOWER Mode
    button.data=true;
    mode_pub.publish(mode);
    click_pub.publish(button);
  }
  else
  {
    button.data=false;
    click_pub.publish(button);
  }

  if (joy->buttons[6])
  {
	go.data =99;
	elevator_arrived_pub.publish(go);
  }


}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_control");
  joy_control joy_control;

  ros::spin();
  return 0;
}
