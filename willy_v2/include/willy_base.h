#ifndef __WILLY_BASE__
#define __WILLY_BASE__

#include <stdlib.h>
#include "ros/ros.h"
#include "serial/serial.h"
#include <string>
#include <geometry_msgs/Twist.h>
#include <willy_v2/Float_Header.h>
#include <unistd.h>
#include <std_msgs/Int32.h>
#include <tf/transform_broadcaster.h>

#define PI 3.14159
#define rad2rev 0.1591549
#define sec2min 0.0166667
#define deg2rad 0.0174533
 

class willy_base {
	public:
		willy_base();
        void run();
	private:			
		ros::NodeHandle n;
        ros::Publisher R_vel_pub, L_vel_pub, motor_vl_pub, motor_vr_pub,vr_temp_pub,vl_temp_pub,vr_cmd_pub,vl_cmd_pub,vr_load_pub,vl_load_pub;
		ros::Subscriber cmd_sub;
		ros::Time curr_T, last_T;
		

		unsigned short calc_crc(unsigned char *buf, int length);
		double hex2dis(unsigned char *data);
		int Serial_set(int Baudrate, std::string Port, int Parity);
        void callback(const geometry_msgs::Twist::ConstPtr &cmd);
        void calculate_cmd(int vr,int vl);
		void send_cmd(unsigned char ID, unsigned short cmd_startpos_, unsigned int cmd_data);
		void send_cmd_single(unsigned char ID, unsigned short cmd_pos_, unsigned short cmd_data);
		int read_cmd(unsigned char ID, unsigned short cmd_startpos_, unsigned short num);

        
        bool is_motor_set;
		int baudrate, hz, Gear_Ratio, last_vl_dir, last_vr_dir, C35;
		double Wheel_Radius, Wheel_Base;
        double linear_vel, angular_vel, VR, VL;
		std::string port;
        unsigned short crc;
		unsigned char ID_1, ID_2, FC, cmd_numx2;
		unsigned short cmd_num;
        unsigned char last_cmd_counter, cmd_counter;
        unsigned short cmd_startpos;
		unsigned int vel_data_1, vel_data_2;
		serial::Serial ser;
		std_msgs::Int32 vr_vel, vl_vel,vr_temp,vl_temp,vr_cmd,vl_cmd,vr_load,vl_load;
};

#endif
