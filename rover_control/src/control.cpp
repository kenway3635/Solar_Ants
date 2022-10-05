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
    linear_vel_fb = msg->twist.twist.linear.x;
    angular_vel_fb = msg->twist.twist.angular.z;
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

float odom_omega(0), imu_omega(0);
ros::Subscriber imu_sub, encoder_sub;

void encoderCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_omega = msg->twist.twist.angular.z;
}

 void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
 imu_omega=msg->angular_velocity.z;
}

float slip_ratio(float enhance)
{
  static std::deque<float> avgloss(10);
  float ratio, avg;
  //float ratio = 1- abs(imu_omega - odom_omega) * enhance;
  if (odom_omega && imu_omega)
    ratio = std::min(abs(odom_omega/imu_omega), abs(imu_omega/odom_omega)) * enhance;
  else
    ratio = 1;
  ratio = std::max(ratio, (float)0.0);
  avgloss.push_front(ratio);
  avgloss.pop_back();
  for (auto &i:avgloss)
    avg +=i;
  avg = avg/10;
  return avg;
}


float PID(float vin, float mea,float kp,float ki,float kd)
{

  //float timeChange=0.3;
  float err = vin-mea;
  static float errSum(0), last_err(0);
  static decltype(ros::Time::now()) last_T(0);

  decltype(ros::Time::now()) curr_T = ros::Time::now();
  
  float dt = (curr_T - last_T).toSec();
  
  errSum += (err * dt);
  double dErr = (err - last_err) / dt;
  last_err = err;
  last_T = curr_T;
   /*Compute PID Output*/
  float u=kp * err + ki * errSum + kd * dErr;
  if (abs(u < 0.06))
    u=0;
  
  return u;
}


float tf_velocity(float velocity = 0, float bound = 1)//check bound of the input velocity
{ 
  if (abs(velocity)<0.06)
  {
    if (velocity<0)
      velocity = -0.06;
    else if (velocity>0)
      velocity=0.06;
  }
  else if (abs(velocity)>bound)
  {
    if (velocity>0)
      velocity = bound;
    else if (velocity <0)
      velocity = - bound;
  }
  return velocity;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "rover_control");
  ros::NodeHandle n;
  ros::Time::init();
  ros::Rate r(30);

  vel_pub =  n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  cmd_stop =  n.advertise<std_msgs::Bool>("cmd_stop",1);
  front_detect_pub = n.advertise<std_msgs::Bool>("front_detect",1);
  vel_sub = n.subscribe("new_cmd_vel", 1, velCallback);
  vel_camera = n.subscribe("visual_cmd_vel", 1, Cam_velCallback);
  visualSW_pub = n.advertise<std_msgs::Bool>("visualSW",1);

  imu_sub = n.subscribe("imu/data_raw", 10, imuCallback);
  encoder_sub = n.subscribe("odom", 1, encoderCallback);
  //-----------------------------------------------------------------------------
  front_right = n.subscribe("front_right_ir",10,IRCallback1);
  front_left = n.subscribe("front_left_ir",10,IRCallback2);
  back_right = n.subscribe("back_right_ir",10,IRCallback3);
  back_left = n.subscribe("back_left_ir",10,IRCallback4);
  //----------------------------------------------------------------------------
  mode_sub=n.subscribe("/Mode",1,Mode);
  click_sub=n.subscribe("click",1,clickCallback);
  odom_sub = n.subscribe("odometry/filtered", 1, odomCallback);
  pub_pose = n.advertise<geometry_msgs::Pose2D>("pose2d", 10);


float last_linear_vel(0),last_angular_vel(0), emergency_brake_vel(0),slip_gain(0), enhance_factor(0);//save last velocity value for recovery
bool stamp(0),visual_stamp(0),ir_stamp(0);
char last_trig, trig_temp; 
auto &detect = front_detect.data;
detect = false;
double not_trigger(0),IR_trigger(0),duration(0), stop_time(0), recovery_vel(0),kp(1),ki(0),kd(0);
bool anti_skid(0);

n.getParam("/rover_control/time_to_stop",stop_time);
n.getParam("/rover_control/recovery_vel",recovery_vel);
n.getParam("/rover_control/kp",kp);
n.getParam("/rover_control/ki",ki);
n.getParam("/rover_control/kd",kd);
n.getParam("/rover_control/anti_skid",anti_skid);
n.getParam("/rover_control/emergency_brake_ratio",emergency_brake_vel);
n.getParam("/rover_control/enhance_factor",enhance_factor);

visualSW_data.data = false;

while (ros::ok())
  {
      slip_gain = slip_ratio(enhance_factor);
      ROS_INFO("last trig = %c", last_trig);
      detect = false;
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
        if ((FL==true)||(FR==true)||(BL==true)||(BR==true))//cliff detected
          {
            IR_trigger = ros::Time::now().toSec();
            duration = (double)(IR_trigger-not_trigger);
            
            //ROS_INFO("trigger duration time = %f",duration);
            //ROS_INFO("cliff  detected!!, doing self recovery");
            //new_vel.linear.x = go_minimum(last_linear_vel*(stop_time-duration)/stop_time);
            //new_vel.angular.z = go_minimum(last_angular_vel*(stop_time-duration)/stop_time);
            new_vel.linear.x = last_linear_vel;
            new_vel.angular.z = last_angular_vel;

            if (duration > stop_time)
            {
              stamp = true;
              ROS_INFO("cliff  detected!!, doing self recovery");
              if (ir_stamp)
              { 
                //ROS_INFO("reset speed to 0");
                //stop.data=true;
                new_vel.linear.x =0;
                new_vel.angular.z=0;
                vel_pub.publish(new_vel);
                ros::Duration(1).sleep();
                ir_stamp = false;
              }
              
              else if((last_trig == 'b' && FL)||( last_trig == 'f' &&BL))
              {
                new_vel.linear.x =0;
                new_vel.angular.z=-recovery_vel*2;
              }
              else if((last_trig == 'B' &&FR)||(last_trig == 'F'&&BR))
              {
                new_vel.linear.x =0;
                new_vel.angular.z=recovery_vel*2;
              }
              else if (FL||FR)
              {
                new_vel.linear.x =-recovery_vel;
                new_vel.angular.z=0;
              }
              else if(BL||BR)
              { 
                new_vel.linear.x =recovery_vel;
                new_vel.angular.z=0;
              }
              
              if ((FR)&&(FL))
                detect = true;
              else
                detect = false;
          
              if (FL)
                trig_temp = 'f';
              else if (FR)
                  trig_temp = 'F';
              else if (BL)
                  trig_temp = 'b';
              else if (BR)
                  trig_temp = 'B'; 
            }
          }
        else
        {
          ir_stamp = true;
          last_trig = trig_temp;
          not_trigger = ros::Time::now().toSec();
          if (stamp == true)
          {
            //stop.data=true;
            //ROS_INFO("reset speed to 0(in board)");
            new_vel.linear.x = 0;
            new_vel.angular.z= 0;
            vel_pub.publish(new_vel);
            ros::Duration(1).sleep();
            stamp = false;
          }
          else if(mode == 1)//manual control AMR
          {
            ROS_INFO("Manual");
            new_vel.linear.x = Vel_x;
            new_vel.angular.z= Ang_z;
            last_linear_vel = Vel_x;
            last_angular_vel = Ang_z;
            if (visual_stamp)  // for reset visualSW data
            {
              visual_stamp = false;
              visualSW_data.data = false;
              visualSW_pub.publish(visualSW_data);
            }
          }
          else if(mode == 0)//camera control
          {
            if (visual_stamp == false)
            {
                visual_stamp = true;
                visualSW_data.data = true;
                visualSW_pub.publish(visualSW_data);
            }
            
            //ROS_INFO("camera mode !");
            if (detect)
            {
              continue;
            }
            last_linear_vel=cam_vel_x;
            last_angular_vel=cam_ang_z;
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
  ROS_INFO("new velocity is %f,  %f",new_vel.linear.x,new_vel.angular.z);
  //ROS_INFO("PID x is %f, PID z is %f", PID(new_vel.linear.x, linear_vel_fb, 1,0,0), PID(new_vel.angular.z, angular_vel_fb, 1,0,0));
  ROS_INFO("linear_vel_fb is %f, angular_vel_fb is %f", linear_vel_fb, angular_vel_fb);
  ROS_INFO("slip gain is %f",slip_gain);
  //ROS_INFO("linear_slip_gain is %f, angular_slip_gain is %f", slip_ratio(new_vel.linear.x, linear_vel_fb), slip_ratio(new_vel.angular.z, angular_vel_fb));
  //new_vel.linear.x = PID(new_vel.linear.x, linear_vel_fb, kp,ki,kd);
  //new_vel.angular.z = PID(new_vel.angular.z, angular_vel_fb, kp,ki,kd);
  new_vel.linear.x = tf_velocity(new_vel.linear.x, 0.4948);
  new_vel.angular.z = tf_velocity(new_vel.angular.z, 1.94) ;

  if (anti_skid)
  {
    if (slip_gain<emergency_brake_vel)
    {
      new_vel.linear.x = 0;
      new_vel.angular.z = 0;
      stop.data = true;
    }
    // else
    // {
    //   new_vel.linear.x = new_vel.linear.x * slip_gain;
    //   new_vel.angular.z = new_vel.angular.z * slip_gain;
    // }
    
  }
 

  vel_pub.publish(new_vel);
  cmd_stop.publish(stop);
  front_detect_pub.publish(front_detect);

  ros::spinOnce();
  r.sleep();
  
  }
  return 0;
}
