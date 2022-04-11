#ifndef __WILLY_BASE__
#define __WILLY_BASE__

#include <stdlib.h>
#include "ros/ros.h"
#include "serial/serial.h"
#include <string>
#include <willy_v2/Float_Header.h>
#include <unistd.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

#define PI 3.14159
#define rad2rev 0.1591549
#define sec2min 0.0166667
#define deg2rad 0.0174533
 

class willy_motor {
	public:
		willy_motor();
        void run();
	private:			
		ros::NodeHandle n;
        ros::Publisher  motor_pub,temp_pub,cmd_pub,load_pub;
		ros::Subscriber cmd_sub,cmd_stop;
		ros::Time curr_T, last_T;

		

		unsigned short calc_crc(unsigned char *buf, int length);
		double hex2dis(unsigned char *data);
		int Serial_set(int Baudrate, std::string Port, int Parity);
        void callback(const std_msgs::Int32MultiArray::ConstPtr& cmd);
		void Stop(const std_msgs::Bool::ConstPtr& msg);
        int calculate_cmd(int v);
		void send_cmd(unsigned char ID, unsigned short cmd_startpos_, unsigned int cmd_data);
		void send_cmd_single(unsigned char ID, unsigned short cmd_pos_, unsigned short cmd_data);
		int read_cmd(unsigned char ID, unsigned short cmd_startpos_, unsigned short num);

        
        bool is_motor_set,stop_l,stop_r;
		int baudrate, hz, Gear_Ratio, last_dir, C35,vel,mode;
		float VL,VR;
		double Wheel_Radius, Wheel_Base;
        double linear_vel, angular_vel;
		std::string port;
        unsigned short crc;
		unsigned char ID_1, ID_2, FC, cmd_numx2;
		unsigned short cmd_num;
        unsigned char last_cmd_counter, cmd_counter;
        unsigned short cmd_startpos;
		unsigned int vel_data;
		serial::Serial ser;
		std_msgs::Int32 vr_vel, vl_vel,vr_temp,vl_temp,vr_cmd,vl_cmd,vr_load,vl_load;
		
};

#endif
