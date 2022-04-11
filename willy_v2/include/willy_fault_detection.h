#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <willy_v2/Float_Header.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


geometry_msgs::Twist vel_msg;

int count;

void Right_detection(const willy_v2::Float_Header::ConstPtr& msg_Right_cmd, const willy_v2::Float_Header::ConstPtr& msg_Right_rsp, const willy_v2::Float_Header::ConstPtr& msg_Left_cmd, const willy_v2::Float_Header::ConstPtr& msg_Left_rsp);

