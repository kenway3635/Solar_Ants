#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include<iostream>

using namespace std;
int main(int argc, char **argv)
{}
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::Publisher command_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Subscriber sub = n.subscribe("commandFromMQTT", 1000, commandCallback);
    ros::Rate loop_rate(10);
}

void commandCallback(const std_msgs::String::ConstPtr& msg)
{
    cout << msg->data << endl;
}