#include <rover.h>
#include "ros_node.h"
#include "time.h"
#include <locale.h>


float slip_ratio(float enhance, float odom_omega, float imu_omega)
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
setlocale( LC_ALL, "" ); 
ros::init(argc, argv, "rover_control");
ros_node node;
auto n = node.n;
node.init_node();

float last_linear_vel(0),last_angular_vel(0), emergency_brake_vel(0),slip_gain(0), enhance_factor(0);//save last velocity value for recovery
bool stamp(0),visual_stamp(0),ir_stamp(0);
string last_trig, trig_temp; 

float  stop_time(0), recovery_vel(0),kp(1),ki(0),kd(0);
double not_trigger(0),IR_trigger(0),duration(0);
bool anti_skid(0);

n.getParam("/rover_control/time_to_stop",stop_time);
n.getParam("/rover_control/recovery_vel",recovery_vel);
n.getParam("/rover_control/kp",kp);
n.getParam("/rover_control/ki",ki);
n.getParam("/rover_control/kd",kd);
n.getParam("/rover_control/anti_skid",anti_skid);
n.getParam("/rover_control/emergency_brake_ratio",emergency_brake_vel);
n.getParam("/rover_control/enhance_factor",enhance_factor);

while (ros::ok())
  {
      slip_gain = slip_ratio(enhance_factor, node.odom_omega, node.imu_omega);
      //ROS_INFO("last trig = %s", last_trig.c_str());
      node.detect = false;
      //ROS_INFO(" time = %f",node.get_time());
      if(node.mode == 99)//emergency stop AMR
      {
        node.stop_data=true;
        node.output_velocity = {0,0};
        ROS_INFO("Stop!");
      }
      else
      {
        node.stop_data=false;
        if (node.ir_detect[0]||node.ir_detect[1]||node.ir_detect[2]||node.ir_detect[3])//cliff detected
          {
            IR_trigger = node.get_time();
            duration = (IR_trigger-not_trigger);
            
            //ROS_INFO("trigger duration time = %f",duration);
            //ROS_INFO("IR trigger time is  = %f",IR_trigger);
            //ROS_INFO("not trigger time is  = %f",not_trigger);
            //ROS_INFO("cliff  detected!!, doing self recovery");
            //new_vel.linear.x = go_minimum(last_linear_vel*(stop_time-duration)/stop_time);
            //new_vel.angular.z = go_minimum(last_angular_vel*(stop_time-duration)/stop_time);

            node.output_velocity = {last_linear_vel, last_angular_vel};

            if (duration > stop_time)
            {
              stamp = true;
              ROS_INFO("cliff  detected!!, doing self recovery");
              if (ir_stamp)
              { 
                //ROS_INFO("reset speed to 0");
                //stop.data=true;
                node.output_velocity = {0,0};
                node.update();
                node._sleep(1);
                ir_stamp = false;
              }
              
              else if((last_trig == "BL" && node.ir_detect[0])||( last_trig == "FL" &&node.ir_detect[2]))
              {
                node.output_velocity = {0,-recovery_vel*2};
              }
              else if((last_trig == "BR" &&node.ir_detect[1])||(last_trig == "FR"&&node.ir_detect[3]))
              {
                node.output_velocity = {0,recovery_vel*2};
              }
              else if (node.ir_detect[1]||node.ir_detect[0])
              {
                node.output_velocity = {-recovery_vel,0};
              }
              else if(node.ir_detect[3]||node.ir_detect[2])
              { 
                node.output_velocity = {recovery_vel,0};
              }
              
              if ((node.ir_detect[1])&&(node.ir_detect[0]))
                node.detect = true;
              else
                node.detect = false;
          
              if (node.ir_detect[0])
                trig_temp = "FL";
              else if (node.ir_detect[1])
                  trig_temp = "FR";
              else if (node.ir_detect[2])
                  trig_temp = "BL";
              else if (node.ir_detect[3])
                  trig_temp = "BR"; 
              node.update();
            }
          }
        else
        {
          ir_stamp = true;
          last_trig = trig_temp;
          not_trigger = node.get_time();
          if (stamp == true)
          {
            //stop.data=true;
            //ROS_INFO("reset speed to 0(in board)");
            node.output_velocity = {0,0};
            node.update();
            ros::Duration(1).sleep();
            stamp = false;
          }
          else if(node.mode == 1)//manual control AMR
          {
            ROS_INFO("Manual");
            node.output_velocity = node.joy_velocity;
            last_linear_vel = node.joy_velocity[0];
            last_angular_vel = node.joy_velocity[1];
            if (visual_stamp)  // for reset visualSW data
            {
              visual_stamp = false;
              node.visualSW = false;
            }
          }
          else if(node.mode == 0)//camera control
          {
            if (visual_stamp == false)
            {
                visual_stamp = true;
                node.visualSW = true;
            }
            
            ROS_INFO("camera mode !");
            if (node.detect)
            {
              continue;
            }
            node.output_velocity = node.camera_velocity;
            last_linear_vel = node.camera_velocity[0];
            last_angular_vel = node.camera_velocity[1];
          }
          else//stationary mode

          {
            node.stop_data=true;
            ROS_INFO("--------------------Press アニキ To Start------------------");
            node.output_velocity = {0,0};
          }
        }
    }
  //ROS_INFO("new velocity is %f,  %f",new_vel.linear.x,new_vel.angular.z);
  //ROS_INFO("PID x is %f, PID z is %f", PID(new_vel.linear.x, linear_vel_fb, 1,0,0), PID(new_vel.angular.z, angular_vel_fb, 1,0,0));
  //ROS_INFO("linear_vel_fb is %f, angular_vel_fb is %f", linear_vel_fb, angular_vel_fb);
  //OS_INFO("slip gain is %f",slip_gain);
  //ROS_INFO("linear_slip_gain is %f, angular_slip_gain is %f", slip_ratio(new_vel.linear.x, linear_vel_fb), slip_ratio(new_vel.angular.z, angular_vel_fb));
  //new_vel.linear.x = PID(new_vel.linear.x, linear_vel_fb, kp,ki,kd);
  //new_vel.angular.z = PID(new_vel.angular.z, angular_vel_fb, kp,ki,kd);
  node.output_velocity[0] = tf_velocity(node.output_velocity[0], 0.4948);
  node.output_velocity[1] = tf_velocity(node.output_velocity[1], 1.94) ;

  if (anti_skid)
  {
    if (slip_gain<emergency_brake_vel)
    {
      node.output_velocity = {0,0};
      node.stop_data = true;
    }
    // else
    // {
    //   new_vel.linear.x = new_vel.linear.x * slip_gain;
    //   new_vel.angular.z = new_vel.angular.z * slip_gain;
    // }
    
  }
 

  node.update();
  
  }
  return 0;
}
