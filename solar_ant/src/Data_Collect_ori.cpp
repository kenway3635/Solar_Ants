#include "ros/ros.h"
#include "std_msgs/String.h"
#include "string"
#include "fstream"
#include "time.h"

void csv_Init()
{
	std::ofstream 
		IMUf("/home/ubuntu/catkin_ws/src/solar_ant/Data/IMU_Data.csv"),
		IRf("/home/ubuntu/catkin_ws/src/solar_ant/Data/IR_Data.csv"),
	       	LM35f("/home/ubuntu/catkin_ws/src/solar_ant/Data/LM35_Data.csv"),	
		GPSf("/home/ubuntu/catkin_ws/src/solar_ant/Data/GPS_Data.csv"),
		Motorf("/home/ubuntu/catkin_ws/src/solar_ant/Data/Motor_Data.csv"),
		Brushf("/home/ubuntu/catkin_ws/src/solar_ant/Data/Brush_Data.csv");

	IMUf << "time, x, y, z" << "\n";
	IRf << "time, part, dist" << "\n";
	LM35f << "time, teperature" << "\n";
	GPSf << "time, add" << "\n";
	Motorf << "time, speed" << "\n";
	Brushf << "time, speed" << "\n";
	
}

void write_csv(std::string filename, char* time, const std_msgs::String::ConstPtr& msg)
{
	//open the destination file to write
	std::ofstream destn_file(filename, std::ios::app);
	// Send data to the stream
	destn_file << time << ", " << msg->data.c_str() << "\n";
        //destn_file << msg->data.c_str() << "\n";
	//Close the file
	destn_file.close();

	return;
}

char* Data_time()
{
	time_t t = time(NULL);
	struct tm* timeinfo = localtime (&t);
	char* time = asctime(timeinfo);
	time[strlen(time)-1] = '\0';
	return time;
}

void IMU_Collect(const std_msgs::String::ConstPtr& msg)
{
	char* time;
	time = Data_time();
	ROS_INFO("I heard: [%s]", msg->data.c_str());
	write_csv("/home/ubuntu/catkin_ws/src/solar_ant/Data/IMU_Data.csv", time, msg);
}

void IR_Collect(const std_msgs::String::ConstPtr& msg)
{ 
	char* time;
        time = Data_time();
	ROS_INFO("I heard: [%s]", msg->data.c_str());
        write_csv("/home/ubuntu/catkin_ws/src/solar_ant/Data/IR_Data.csv", time, msg);
} 

void LM35_Collect(const std_msgs::String::ConstPtr& msg)
{
        char* time;
        time = Data_time();
        ROS_INFO("I heard: [%s]", msg->data.c_str());
        write_csv("/home/ubuntu/catkin_ws/src/solar_ant/Data/LM35_Data.csv", time, msg);
}

void GPS_Collect(const std_msgs::String::ConstPtr& msg)
{
	char* time;
        time = Data_time();
        write_csv("/home/ubuntu/catkin_ws/src/solar_ant/Data/GPS_Data.csv", time, msg);
}

void Motor_Collect(const std_msgs::String::ConstPtr& msg)
{
	char* time;
        time = Data_time();
        write_csv("/home/ubuntu/catkin_ws/src/solar_ant/Data/Motor_Data.csv", time, msg);
}

void Brush_Collect(const std_msgs::String::ConstPtr& msg)
{
	char* time;
        time = Data_time();
        write_csv("/home/ubuntu/catkin_ws/src/solar_ant/Data/Brush_Data.csv", time, msg);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Data_Collect");

	ros::NodeHandle n;
	csv_Init();

	ros::Subscriber IMU_sub = n.subscribe("IMU", 1000, IMU_Collect);
	ros::Subscriber IR_sub = n.subscribe("IR_Sensor", 1000, IR_Collect);
	ros::Subscriber LM35_sub = n.subscribe("LM35", 1000, LM35_Collect);
	ros::Subscriber GPS_sub = n.subscribe("GPS", 1000, GPS_Collect);
	ros::Subscriber Motor_sub = n.subscribe("Motor", 1000, Motor_Collect);
	ros::Subscriber Brush_sub = n.subscribe("Brush", 1000, Brush_Collect);

	ros::spin();

	return 0;
}
