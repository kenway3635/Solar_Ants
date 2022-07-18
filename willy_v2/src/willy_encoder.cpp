#include <willy_encoder.h>




int motor_vl=0, motor_vr = 0;



void motor_vl_callback(const std_msgs::Int32::ConstPtr &data) 
{
	motor_vl = data->data;

}
void motor_vr_callback(const std_msgs::Int32::ConstPtr &data) 
{
	motor_vr = data->data;
}



int main(int argc, char **argv) 
{
	ros::init(argc, argv, "willy_v2_encoder");
	ros::NodeHandle n;
	ros::NodeHandle nh("~");
	ros::Time Curr_T;
	ros::Time Last_T = ros::Time::now();
	


	
	ros::Rate loop_rate(20.0);
	size_t now_read = 0;
	int count = 0;
	unsigned char R_En[4], L_En[4];
	double Curr_R_rad, Curr_L_rad, Last_R_rad, Last_L_rad;
	double vr, vl, dt, v_m, yaw_rate, d_x, d_y, d_phi, x = 0, y = 0, phi = 0;
	nav_msgs::Odometry odom;
    geometry_msgs::Quaternion odom_quat;
	//willy_v2::Float_Header VL_msg, VR_msg, Left_rad_msg, Right_rad_msg;
	float Curr_VL,Curr_VR,Last_VL,Last_VR;

	ros::Publisher odom_pub= n.advertise<nav_msgs::Odometry>("odom", 20);
	//ros::Publisher WL_pub = n.advertise<willy_v2::Float_Header>("Left_Vel_Response", 10);
	//ros::Publisher WR_pub = n.advertise<willy_v2::Float_Header>("Right_Vel_Response", 10);
	//ros::Publisher Left_rad_pub = n.advertise<willy_v2::Float_Header>("Left_Rad_Response", 10);
	//ros::Publisher Right_rad_pub = n.advertise<willy_v2::Float_Header>("Right_Rad_Response", 10);
	ros::Subscriber motor_vl_sub = n.subscribe("/motor_vl", 5, motor_vl_callback);
	ros::Subscriber motor_vr_sub = n.subscribe("/motor_vr", 5, motor_vr_callback);
	
	while(ros::ok()) 
	{
		//printf("=========== round %d ===========\n",);
		count++;
		Curr_T = ros::Time::now();
		

		if (count > 1){
			dt = (Curr_T - Last_T).toSec();
			Curr_VL = (motor_vl / gear_R)*Wheel_R/rad2rev/60;
			Curr_VR = (motor_vr / gear_R)*Wheel_R/rad2rev/60;

			//printf("dt is: %lf\n", dt);
			if (Curr_VL > 0 && Last_VL > 0 ){ 
				vl = (Curr_VL + Last_VL)/2;				
			}
			else if (Curr_VL < 0 && Last_VL < 0){
				vl = (Curr_VL + Last_VL)/2;
			}
			else{
				vl = 0;
			}

			if (Curr_VR > 0 && Last_VR > 0 ){ 
				vr = (Curr_VR + Last_VR)/2;
			}
			else if (Curr_VR < 0 && Last_VR < 0){
				vr = (Curr_VR + Last_VR)/2;
			}
			else{
				vr = 0;
			}


			v_m = (vl + vr)/2;
			yaw_rate = (vr - vl)/Wheel_B;
			d_x = v_m*cos(phi)*dt;
			d_y = v_m*sin(phi)*dt;
			d_phi = yaw_rate*dt;
			x += d_x;
			y += d_y;
			phi += d_phi;
			ROS_INFO("X: %f, Y: %f, Phi: %f.", x, y, phi);
		
			

			//VL_msg.header.stamp = Curr_T;
			//VR_msg.header.stamp = Curr_T;
			//Left_rad_msg.header.stamp = Curr_T;
			//Right_rad_msg.header.stamp = Curr_T;
			
			//VL_msg.num = (Curr_L_rad - Last_L_rad)/dt*rad2rev*60*gear_R;
			//VR_msg.num = (Curr_R_rad - Last_R_rad)/dt*rad2rev*60*gear_R;
			//Left_rad_msg.num = Curr_L_rad;
			//Right_rad_msg.num = Curr_R_rad;
			
			odom_quat = tf::createQuaternionMsgFromYaw(phi);
			odom.header.stamp = Curr_T;
			odom.header.frame_id = "odom";

			//set the position
			odom.pose.pose.position.x = x;
			odom.pose.pose.position.y = y;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;
			odom.pose.covariance[0] = 0.0;
			odom.pose.covariance[7] = 0.0;
			odom.pose.covariance[35] = 1.0;
			//set the velocity
			odom.child_frame_id = "base_link";
			odom.twist.twist.linear.x = v_m;
			odom.twist.twist.linear.y = 0;
			odom.twist.twist.angular.z = yaw_rate;
			odom.twist.covariance[0] = 0.1;
			odom.twist.covariance[7] = 0.0001;
			odom.twist.covariance[35] = 1.0;
			//publish the message
			odom_pub.publish(odom);
			
			//WL_pub.publish(VL_msg);
			//WR_pub.publish(VR_msg);
			//Left_rad_pub.publish(Left_rad_msg);
			//Right_rad_pub.publish(Right_rad_msg);

			Last_VL = Curr_VL;
			Last_VR = Curr_VR;
				

		}
		
		Last_T = Curr_T;


		// odom 
		static tf::TransformBroadcaster odom_footprint;
		static tf::TransformBroadcaster footprint_baselink;
		static tf::TransformBroadcaster imu;
	   	tf::Transform transform;
		tf::Quaternion q;
		//ros::Rate loop_rate(20);
		//while(ros::ok()) {
		/*
		transform.setOrigin(tf::Vector3(0.0, 0.0, 0.1));
		q.setRPY(0, 0, 0);
		transform.setRotation(q);
 		footprint_baselink.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "base_link"));
		*/

		// q.setRPY(0, 0, phi);
		// transform.setOrigin(tf::Vector3(x, y, 0.0));
		// transform.setRotation(q);
		// odom_footprint.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

		q.setRPY(0, 0, 0);
		transform.setOrigin(tf::Vector3(0.16, -0.10, 0.1));
		transform.setRotation(q);
		imu.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link","imu_link"));

		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
