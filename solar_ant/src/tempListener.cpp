#include "ros/ros.h"
#include "std_msgs/String.h"
#include "string"
#include "fstream"
#include "time.h"

void csv_Init()
{
	std::ofstream 
	  LM35f_1("LM35_1_Data.csv"),	//brush motor temperature
		LM35f_2("LM35_2_Data.csv"),//DC-DC converter temperature
		LM35f_3("LM35_3_Data.csv"),//wheel motor temperature
		LM35f_4("LM35_4_Data.csv"),	//rapsberry pi temperature
    LM35f_5("LM35_5_Data.csv");	//room temperature

	LM35f_1 << "time, teperature" << "\n";
	LM35f_2 << "time, teperature" << "\n";
	LM35f_3 << "time, teperature" << "\n";
	LM35f_4 << "time, teperature" << "\n";
  LM35f_5 << "time, teperature" << "\n";
	
}
void write_csv(std::string filename, char* time, const std_msgs::String::ConstPtr& msg)
{
	//open the destination file to write
	std::ofstream destn_file(filename, std::ios::app);
	// Send data to the stream
	destn_file << time << ", " << msg->data.c_str() << "\n";
       // destn_file << msg->data.c_str() << "\n";
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

void temp1Callback(const std_msgs::String::ConstPtr& msg)
{
  char* time;
  time = Data_time();
  ROS_INFO("Brush motor = %s, ", msg->data.c_str());
  write_csv("LM35_1_Data.csv", time, msg);
}
void temp2Callback(const std_msgs::String::ConstPtr& msg)
{
    char* time;
  time = Data_time();
  ROS_INFO("DC converter= %s, ", msg->data.c_str());
  write_csv("LM35_2_Data.csv", time, msg);
}
void temp3Callback(const std_msgs::String::ConstPtr& msg)
{
  char* time;
  time = Data_time();
  ROS_INFO("wheel motor= %s, ", msg->data.c_str());
  write_csv("LM35_3_Data.csv", time, msg);
}
void temp4Callback(const std_msgs::String::ConstPtr& msg)
{
  char* time;
  time = Data_time();
  ROS_INFO("Rpi= %s, ",msg->data.c_str());
  write_csv("LM35_4_Data.csv", time, msg);
}
void temp5Callback(const std_msgs::String::ConstPtr& msg)
{
  char* time;
  time = Data_time();
  ROS_INFO("Battery= %s, ",msg->data.c_str());
  write_csv("LM35_5_Data.csv", time, msg);
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
   csv_Init();
ros::Subscriber sub1 = n.subscribe("temp_01", 1000, temp1Callback);
ros::Subscriber sub2 = n.subscribe("temp_02", 1000, temp2Callback);
 ros::Subscriber sub3 = n.subscribe("temp_03", 1000, temp3Callback);
ros::Subscriber sub4 = n.subscribe("temp_04", 1000, temp4Callback);
ros::Subscriber sub5 = n.subscribe("temp_05", 1000, temp5Callback);
   ros::spin();
  return 0;
}
