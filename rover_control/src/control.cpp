#include <rover.h>



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
int state =1;
float length=0.7;
float wide = 0.1;
float error_x,error_y,error_theta;
float error_linear_thresh=0.05;
float error_angular_thresh=0.1;
ros::Time::init();
ros::Rate r(10);
bool button_reg=0;//save state of  button 
while (ros::ok())
  {
    //ROS_INFO("button_reg= %d",button_reg);
    //ROS_INFO("x= %f, y=%f, theta=%f",x,y,theta);
    if (((FL==true)||(FR==true)||(BL==true)||(BR==true))&&(button_reg==0))//cliff stop state
      {
        ROS_INFO("cliff  detected!!, press any button to continue, will switch to manual mode");
        stop.data=true;
        new_vel.linear.x = 0;
        new_vel.angular.z= 0;
        if(button)
        {
          button_reg=1;
          stop.data=false;
        }
      }
    else
    {
       if ((FL==false)&&(FR==false)&&(BL==false)&&(BR==false))//when ir back to ground, reset cliff state
       {
        button_reg=0;
       }
      
      if(mode == 99)//stop AMR
      {
        stop.data=true;
        ROS_INFO("Stop!");
      }
      else if(mode == 1)//manual control AMR
      {
        stop.data=false;
        ROS_INFO("Manual");
        new_vel.linear.x = Vel_x;
        new_vel.angular.z= Ang_z;
        state=1;
      }
      else if(mode == 2)// AUTO tracking point IMU & encoder
      {
        stop.data=false;
        float Last_err_x=error_x;
        float Last_err_y=error_y;
        float Last_err_theta=error_theta;
        // error_x=length-abs(x-reg_x);
        // error_y=wide-abs(y-reg_y);
        // error_theta=90-abs(theta-reg_theta);
        float linear_kp,linear_ki,linear_kd,angular_kp,angular_ki,angular_kd,angle_upperlim,angle_lowerlim,linear_upperlim,linear_lowerlim;
        linear_kp=0.5;linear_ki=0.5;linear_kd=0;
        angular_kp=0.005;angular_ki=0.003;angular_kd=0;
        angle_upperlim=0.1;angle_lowerlim=0.1;
        linear_upperlim=0.3;linear_lowerlim=0.1;

        switch(state)
        {
          case 1:
            ROS_INFO("x_forward!");
            ROS_INFO("x_err = %f, y_err=%f",error_x,error_y);
            error_x=length-x;
            // error_theta=-0.01*theta;
            new_vel.linear.x=PID(error_x,Last_err_x,linear_kp,linear_ki,linear_kd,linear_upperlim,linear_lowerlim);
            new_vel.angular.z=0;
            if ((abs(error_x)<error_linear_thresh)&&(abs(error_theta)<error_angular_thresh))
            {
            state=2;
            new_vel.linear.x=0;
            new_vel.angular.z=0;
            }
            break;


          case 2:
            ROS_INFO("90deg_turn!");
            ROS_INFO("angle_err = %f, P_theta=%f",error_theta,PID(error_theta,Last_err_theta,angular_kp,angular_ki,angular_kd,angle_upperlim,angle_lowerlim));
            error_theta=90-theta;
            new_vel.linear.x=0;
            new_vel.angular.z=PID(error_theta,Last_err_theta,angular_kp,angular_ki,angular_kd,angle_upperlim,angle_lowerlim);
            if (abs(error_theta)<error_angular_thresh)
            {
              state=3;
              new_vel.linear.x=0;
              new_vel.angular.z=0;
            }
            break;
        

        case 3:
            ROS_INFO("y_forward!");
            ROS_INFO("y_err = %f, P_y=%f",error_y,PID(error_y,Last_err_y,linear_kp,linear_ki,linear_kd,linear_upperlim,linear_lowerlim));
            error_y=wide-y;
            //error_theta=-0.01*(theta-90);
            new_vel.linear.x=PID(error_y,Last_err_y,linear_kp,linear_ki,linear_kd,linear_upperlim,linear_lowerlim);
            new_vel.angular.z=0;
            if (abs(error_y)<error_linear_thresh)
            {
              state=4;
            new_vel.linear.x=0;
            new_vel.angular.z=0;
            }
            break;
        
        
        case 4:
            ROS_INFO("90deg_turn!");
            ROS_INFO("angle_err = %f, P_theta=%f",error_theta,PID(error_theta,Last_err_theta,angular_kp,angular_ki,angular_kd,angle_upperlim,angle_lowerlim));
            if(theta<0){theta=theta+360;}
            error_theta=180-theta;
            new_vel.linear.x=0;
            new_vel.angular.z=PID(error_theta,Last_err_theta,angular_kp,angular_ki,angular_kd,angle_upperlim,angle_lowerlim);
            if (abs(error_theta)<error_angular_thresh)
            {
              state=5;
              new_vel.linear.x=0;
              new_vel.angular.z=0;
            }
            break;
        

          case 5:
            ROS_INFO("x_forward!");
            ROS_INFO("x_err = %f, y_err=%f",error_x,error_y);
            error_x=x;
            //error_theta=-0.01*theta;
            new_vel.linear.x=PID(error_x,Last_err_x,linear_kp,linear_ki,linear_kd,linear_upperlim,linear_lowerlim);
            new_vel.angular.z=0;
            if ((abs(error_x)<error_linear_thresh)&&(abs(error_theta)<error_angular_thresh))
            {
            state=6;
            new_vel.linear.x=0;
            new_vel.angular.z=0;
            }
            break;
          


          case 6:
            ROS_INFO("90deg_turn!");
            ROS_INFO("angle_err = %f, P_theta=%f",error_theta,PID(error_theta,Last_err_theta,angular_kp,angular_ki,angular_kd,angle_upperlim,angle_lowerlim));
            if(theta<0){theta=theta+360;}
            error_theta=270-theta;
            new_vel.linear.x=0;
            new_vel.angular.z=PID(error_theta,Last_err_theta,angular_kp,angular_ki,angular_kd,angle_upperlim,angle_lowerlim);
            if (abs(error_theta)<error_angular_thresh)
            {
              state=7;
              new_vel.linear.x=0;
              new_vel.angular.z=0;
            }
            break;
          

          case 7:
            ROS_INFO("y_forward!");
            ROS_INFO("y_err = %f, P_y=%f",error_y,PID(error_y,Last_err_y,linear_kp,linear_ki,linear_kd,linear_upperlim,linear_lowerlim));
            error_y=y;
            new_vel.linear.x=PID(error_y,Last_err_y,linear_kp,linear_ki,linear_kd,linear_upperlim,linear_lowerlim);
            new_vel.angular.z=0;
            if (abs(error_y)<error_linear_thresh)
            {
            state=8;
            new_vel.linear.x=0;
            new_vel.angular.z=0;
            }
            break;
          
          
          case 8:
            ROS_INFO("90deg_turn!");
            ROS_INFO("angle_err = %f, P_theta=%f",error_theta,PID(error_theta,Last_err_theta,angular_kp,angular_ki,angular_kd,angle_upperlim,angle_lowerlim));
            error_theta=-theta;
            new_vel.linear.x=0;
            new_vel.angular.z=PID(error_theta,Last_err_theta,angular_kp,angular_ki,angular_kd,angle_upperlim,angle_lowerlim);
            if (abs(error_theta)<error_angular_thresh)
            {
              state=9;
              new_vel.linear.x=0;
              new_vel.angular.z=0;
            }
            break;
          case 9:
            ROS_INFO("90deg_turn!");
            ROS_INFO("angle_err = %f, P_theta=%f",error_theta,PID(error_theta,Last_err_theta,angular_kp,angular_ki,angular_kd,angle_upperlim,angle_lowerlim));
            loop++;
            if (loop<3)
            {
              state = 1;
            }
            break;

        }
      }
      else if(mode == 0)//camera control
      {
        stop.data=false;
        ROS_INFO("camera mode !");
        new_vel.linear.x=cam_vel_x;
        new_vel.angular.z=cam_ang_z;
      }
      else//stationary mode
      {
        stop.data=true;
        ROS_INFO("--------------------Press Any Key To Start------------------");
        new_vel.linear.x=0;
        new_vel.angular.z=0;
      }

    }
    
  vel_pub.publish(new_vel);
  cmd_stop.publish(stop);
  ros::spinOnce();
  r.sleep();
  
  }
  return 0;
}
