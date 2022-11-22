#ifndef ROS_NODE
#define ROS_NODE

#include "ros/ros.h"
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Imu.h>
#include <vector>
#include <time.h>

using std::vector;

class ros_node
{
    public:
        vector<float> joy_velocity = {0,0}, camera_velocity={0,0}, output_velocity={0,0};
        vector<int>ir_detect={0,0,0,0};
        int mode = 100;
        bool visualSW = 0, detect = 0, stop_data = 0;
        float odom_omega = 0, imu_omega = 0;
        ros::NodeHandle n;

        double get_time()
        {
            double secs =ros::Time::now().toSec();
            return secs;
        }
        void _sleep(float duration)
        {
            ros::Duration(duration).sleep();
        }
        void init_node()
        {
            ros::Time::init();
            create_topic();
        };
        void update()
        {
            convert_rosmsg();
            pub_pose.publish(pose2d);
            vel_pub.publish(new_vel);
            cmd_stop.publish(stop);
            front_detect_pub.publish(front_detect);
            visualSW_pub.publish(visualSW_data);
            ros::spinOnce();
            r.sleep();
        };

    private:
        
        ros::Rate r = 30;
        geometry_msgs::Twist new_vel;
        std_msgs::Bool stop;
        std_msgs::Bool front_detect;
        std_msgs::Bool visualSW_data;
        geometry_msgs::Pose2D pose2d;
        ros::Publisher vel_pub, cmd_stop, front_detect_pub, pub_pose, visualSW_pub;
        ros::Subscriber vel_sub, vel_camera, front_right, front_left, back_right, back_left, mode_sub, odom_sub, click_sub, imu_sub, encoder_sub;
        void create_topic()
        {
            vel_pub= n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
            cmd_stop = n.advertise<std_msgs::Bool>("cmd_stop",1);
            pub_pose = n.advertise<geometry_msgs::Pose2D>("pose2d", 10);
            visualSW_pub = n.advertise<std_msgs::Bool>("visualSW",1);
            front_detect_pub = n.advertise<std_msgs::Bool>("front_detect",1);
            vel_sub = n.subscribe("new_cmd_vel", 1, &ros_node::velCallback,this);
            vel_camera = n.subscribe("visual_cmd_vel", 1, &ros_node::Cam_velCallback, this);
            front_right = n.subscribe("front_right_ir",10,&ros_node::FR_IRCallback, this);
            front_left = n.subscribe("front_left_ir",10,&ros_node::FL_IRCallback,this);
            back_right = n.subscribe("back_right_ir",10,&ros_node::BR_IRCallback,this);
            back_left = n.subscribe("back_left_ir",10,&ros_node::BL_IRCallback,this);
            mode_sub =n.subscribe("/Mode",1,&ros_node::Mode,this);
            odom_sub = n.subscribe("odometry/filtered", 1, &ros_node::odomCallback,this);
            click_sub =n.subscribe("click",1,&ros_node::clickCallback,this);
            imu_sub = n.subscribe("imu/data_raw", 10, &ros_node::imuCallback,this);
            encoder_sub = n.subscribe("odom", 1, &ros_node::encoderCallback,this);
        };
        void convert_rosmsg()
        {
            front_detect.data = detect;
            stop.data = stop_data;
            new_vel.linear.x = output_velocity[0];
            new_vel.angular.z = output_velocity[1];
            visualSW_data.data = visualSW;

        }
        double rad2deg(double radians)
        {
            return radians * 180.0 / M_PI;
        }

        void FR_IRCallback(const std_msgs::Bool::ConstPtr& msg)
        {
            ir_detect[1]=msg->data;
        }
        void FL_IRCallback(const std_msgs::Bool::ConstPtr& msg)
        {
            ir_detect[0]=msg->data;
        }
        void BR_IRCallback(const std_msgs::Bool::ConstPtr& msg)
        {
            ir_detect[3]=msg->data;
        }
        void BL_IRCallback(const std_msgs::Bool::ConstPtr& msg)
        {
            ir_detect[2]=msg->data;
        }

        void Mode(const std_msgs::Int32::ConstPtr& msg)
        {
            mode=msg->data;
        }

        void velCallback(const geometry_msgs::Twist::ConstPtr& msg)
        {
            joy_velocity[0]=msg->linear.x;
            joy_velocity[1]=msg->angular.z;
        }

        void Cam_velCallback(const geometry_msgs::Twist::ConstPtr& msg)
        {
            camera_velocity [0]=msg->linear.x;
            camera_velocity[1]=msg->angular.z;
        }

        void clickCallback(const std_msgs::Bool::ConstPtr& msg) // not using 
        {
            //button=msg->data;
        }


        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
        {
        pose2d.x = msg->pose.pose.position.x;
        pose2d.y = msg->pose.pose.position.y;

        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
            );
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        pose2d.theta = rad2deg(yaw);
        }
        void encoderCallback(const nav_msgs::Odometry::ConstPtr& msg)
        {
            odom_omega = msg->twist.twist.angular.z;
        }

        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
        {
            imu_omega=msg->angular_velocity.z;
        }


};
#endif