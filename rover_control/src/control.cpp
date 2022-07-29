#include <rover.h>
#include "time.h"


double rad2deg(double radians)
  {
    return radians * 180.0 / M_PI;
  }

void IRCallback1(const std_msgs::Bool::ConstPtr& msg)
{
    //ROS_INFO("distance: [%f]", msg->range);
    FR=msg->data;
}
void IRCallback2(const std_msgs::Bool::ConstPtr& msg)
{
    //ROS_INFO("distance: [%f]", msg->range);
    FL=msg->data;
}
void IRCallback3(const std_msgs::Bool::ConstPtr& msg)
{
    //ROS_INFO("distance: [%f]", msg->range);
    BR=msg->data;
}
void IRCallback4(const std_msgs::Bool::ConstPtr& msg)
{
    //ROS_INFO("distance: [%f]", msg->range);
     BL=msg->data;
}

void Mode(const std_msgs::Int32::ConstPtr& msg)
{
     mode=msg->data;
}

 void velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
 Vel_x=msg->linear.x;
 Ang_z=msg->angular.z;
}

 void Cam_velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
 cam_vel_x=msg->linear.x;
 cam_ang_z=msg->angular.z;
}

 void clickCallback(const std_msgs::Bool::ConstPtr& msg)
{
button=msg->data;
}


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  geometry_msgs::Pose2D pose2d;
  pose2d.x = msg->pose.pose.position.x;
  pose2d.y = msg->pose.pose.position.y;
    
  // ROS_INFO("Seq: [%d]", msg->header.seq);
  // ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  // ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  // ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
      tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
        );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

   x= pose2d.x;
   y= pose2d.y;
   theta=rad2deg(yaw);
  pose2d.theta = theta;
  pub_pose.publish(pose2d);
}

float PID(float err,float Last_err,float kp,float ki,float kd,float upper_lim,float lower_lim)
{
  //Last_T=Curr_T;
 // Curr_T = ros::Time::now();
  //float timeChange = (Curr_T-Last_T).toSec();
  float timeChange=0.1;

  double errSum;
  errSum += (err * timeChange);
   double dErr = (err - Last_err) / timeChange;
   /*Compute PID Output*/
  float u=kp * err + ki * errSum + kd * dErr;

  if(u>0)
  {
   if (abs(u)<lower_lim){u=lower_lim;}//lower velocity limit
   if(abs(u)>upper_lim){u=upper_lim;}//upper velocity limit
  }
  else if(u<0)
  {
   if (abs(u)<lower_lim){u=-lower_lim;}//lower velocity limit
   if(abs(u)>upper_lim){u=-upper_lim;}//upper velocity limit 
  }
  return u;
}

float go_minimum(float velocity)//check minimum of the input command
{
  if (abs(velocity)<0.06)
  {
  if (velocity<0){velocity = -0.06;}
  else if (velocity>0){velocity=0.06;}
  }
  return velocity;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle n;

  vel_pub =  n.advertise<geometry_msgs::Twist>("cmd_vel", 20);
  cmd_stop =  n.advertise<std_msgs::Bool>("cmd_stop",1);
  vel_sub = n.subscribe("new_cmd_vel", 10, velCallback);
  vel_camera = n.subscribe("visual_cmd_vel", 10, Cam_velCallback);
  //-----------------------------------------------------------------------------
  front_right = n.subscribe("front_right_ir",10,IRCallback1);
  front_left = n.subscribe("front_left_ir",10,IRCallback2);
  back_right = n.subscribe("back_right_ir",10,IRCallback3);
  back_left = n.subscribe("back_left_ir",10,IRCallback4);
  //----------------------------------------------------------------------------
mode_sub=n.subscribe("/Mode",1,Mode);
click_sub=n.subscribe("click",1,clickCallback);
odom_sub = n.subscribe("odom", 1, odomCallback);
pub_pose = n.advertise<geometry_msgs::Pose2D>("pose2d", 10);
//---params for odom+imu path planning---//
int state =1;
float length=0.7;
float wide = 0.1;
float error_x,error_y,error_theta;
float error_linear_thresh=0.05;
float error_angular_thresh=0.1;
//---------------------------------------
ros::Time::init();
ros::Rate r(10);
float last_linear_vel=0;//save last velocity value for recovery
float last_angular_vel=0;
bool stamp= false;
bool stop_ir_function = true;

clock_t not_trigger,IR_trigger;
double duration;
float stop_time = 2;

while (ros::ok())
  {
      if(mode == 99)//emergency stop AMR
      {
        stop.data=true;
        new_vel.linear.x = 0;
        new_vel.angular.z = 0;
        ROS_INFO("Stop!");
      }
      else
      {
        stop.data=false;
         if (mode ==1)
        {
          if ((Vel_x != 0) || (Ang_z != 0))
          {
          last_linear_vel = Vel_x;
          last_angular_vel = Ang_z;
          }
        }
        else if (mode ==0)
        {
          if ((cam_vel_x != 0) || (cam_ang_z != 0))
            {
            last_linear_vel=cam_vel_x;
            last_angular_vel=cam_ang_z;
            }
        }


        if (((FL==true)||(FR==true)||(BL==true)||(BR==true)) &&(stop_ir_function == false))//cliff detected
          {
            IR_trigger = clock();
            duration = (double)(IR_trigger-not_trigger)/10000;
            ROS_INFO("trigger duration time = %f",duration);
            //ROS_INFO("cliff  detected!!, doing self recovery");
            stamp = true;
            new_vel.linear.x = go_minimum(last_linear_vel*(stop_time-duration)/stop_time);
            new_vel.angular.z = go_minimum(last_angular_vel*(stop_time-duration)/stop_time);

            if (duration > stop_time)
            {
              if((FL==true)&&(BL==true))
              {
                new_vel.linear.x =0;
                new_vel.angular.z=-0.4;
              }
              else if((FR==true)&&(BR==true))
              {
                new_vel.linear.x =0;
                new_vel.angular.z=0.4;
              }
              else if ((FL==true)||(FR==true))
              {
                new_vel.linear.x =-0.4;
                new_vel.angular.z=0;
              }
              else if((BL==true)||(BR==true))
              { 
                new_vel.linear.x =0.4;
                new_vel.angular.z=0;
              }
            }
          }
        else
        {
          not_trigger = clock();
          if (stamp == true)
          {
            stop.data=true;
            new_vel.linear.x = 0;
            new_vel.angular.z= 0;
            stamp = false;
          }
          else if(mode == 1)//manual control AMR
          {
            ROS_INFO("Manual");
            new_vel.linear.x = Vel_x;
            new_vel.angular.z= Ang_z;
            state=1;
          }
          else if(mode == 0)//camera control
          {
            ROS_INFO("camera mode !");
            new_vel.linear.x=cam_vel_x;
            new_vel.angular.z=cam_ang_z;
          }
          else//stationary mode

          {
            stop.data=true;
            //ROS_INFO("--------------------Press Any Key To Start------------------");
            new_vel.linear.x=0;
            new_vel.angular.z=0;
          }
        }
    }
  //ROS_INFO("new velocity is %f,  %f",new_vel.linear.x,new_vel.angular.z);
 
  vel_pub.publish(new_vel);
  cmd_stop.publish(stop);
  ros::spinOnce();
  r.sleep();
  
  }
  return 0;
}
