#include "ros/ros.h"
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Imu.h>
#include <cmath>
#include "string.h"

using std::string;

ros::Publisher vel_pub;
ros::Publisher cmd_stop;
ros::Publisher front_detect_pub;
ros::Subscriber vel_sub;
ros::Subscriber vel_camera;
ros::Subscriber front_right;
ros::Subscriber front_left;
ros::Subscriber back_right;
ros::Subscriber back_left;
ros::Subscriber mode_sub;
ros::Subscriber odom_sub;
ros::Subscriber click_sub;
ros::Publisher pub_pose;
ros::Publisher visualSW_pub;

geometry_msgs::Twist new_vel;
std_msgs::Bool stop;
std_msgs::Bool front_detect;
std_msgs::Bool visualSW_data;

float Vel_x,Ang_z,cam_vel_x,cam_ang_z;
float linear_vel_fb, angular_vel_fb;
bool FL,FR,BL,BR,cliff,button;
float x,y,l,theta,loop=0;
int mode=50;
