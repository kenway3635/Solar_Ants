#ifndef __WILLY_ENCODER__
#define __WILLY_ENCODER__

#include <stdlib.h>
#include "ros/ros.h"
#include "serial/serial.h"
#include <string>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <willy_v2/Float_Header.h>

#define Wheel_R 0.075    
#define Wheel_B 0.255 //0.31922
#define gear_R 50
#define PI 3.14159
#define rad2rev 0.1591549
#define deg2rad 0.0174533

#endif
