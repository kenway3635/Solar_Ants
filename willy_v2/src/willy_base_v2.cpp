#include <willy_base.h>

willy_base::willy_base() : ID_1(0x01), ID_2(0x02), FC(0x10), cmd_num(0x0002), cmd_numx2(0x04), Gear_Ratio(50), Wheel_Radius(0.075), Wheel_Base(0.255), last_vl_dir(0), last_vr_dir(0), is_motor_set(false), C35(4000), linear_vel(0), angular_vel(0) {

	ROS_INFO("initialize");
	
	ros::NodeHandle nh_private("~");
	nh_private.param<int>("Baudrate", baudrate, 115200);
	nh_private.param<std::string>("Port", port, "/dev/ttyUSB0");
    nh_private.param<int>("control_hz", hz, 10);
	R_vel_pub = n.advertise<willy_v2::Float_Header>("Right_Vel_Cmd", 20);
	L_vel_pub = n.advertise<willy_v2::Float_Header>("Left_Vel_Cmd", 20);
	motor_vl_pub = n.advertise<std_msgs::Int32>("/motor_vl", 20);
	motor_vr_pub = n.advertise<std_msgs::Int32>("/motor_vr", 20);
	cmd_sub = n.subscribe("/cmd_vel", 5, &willy_base::callback, this);

	vr_temp_pub = n.advertise<std_msgs::Int32>("/vr_temp", 20);
	vl_temp_pub = n.advertise<std_msgs::Int32>("/vl_temp", 20);
	vr_cmd_pub = n.advertise<std_msgs::Int32>("/vr_cmd", 20);
	vl_cmd_pub = n.advertise<std_msgs::Int32>("/vl_cmd", 20);
	vr_load_pub = n.advertise<std_msgs::Int32>("/vr_load", 20);
	vl_load_pub = n.advertise<std_msgs::Int32>("/vl_load", 20);

    if(!Serial_set(baudrate, port, 2)) 
    {
		ROS_ERROR_STREAM("Unable to open port");
	}
}

unsigned short willy_base::calc_crc(unsigned char *buf, int length) 
{
	unsigned short crc = 0xFFFF;
	unsigned char LSB;
	for (int i = 0; i < length; i++) {
		crc ^= buf[i];
		for (int j = 0; j < 8; j++) {
			LSB = crc & 1;
   			crc = crc >> 1;
   			if (LSB) {
				crc ^= 0xA001;
   			}
  		}
 	}
 	return crc;
};

double willy_base::hex2dis(unsigned char *data) 
{

	int hi = 0, lo = 0;
	double dis = 0;
	hi = (data[0] << 8) + data[1];
	lo = (data[2] << 8) + data[3];
	dis = ((hi*360 + lo*0.036)*deg2rad)/Gear_Ratio;

	return dis;

};

void willy_base::send_cmd(unsigned char ID, unsigned short cmd_startpos_, unsigned int cmd_data) 
{
	//ROS_INFO("ID = %d, pos = %04X, cmd = %d", ID, cmd_startpos_, cmd_data);
	unsigned char speed_data[11], cmd_crc[13];
	speed_data[0] = ID;
	speed_data[1] = FC;
	speed_data[2] = ((cmd_startpos_ & 0xFF00) >> 8);
	speed_data[3] = (cmd_startpos_ & 0xFF);
	speed_data[4] = ((cmd_num & 0xFF00) >> 8);
	speed_data[5] = (cmd_num & 0xFF);
	speed_data[6] = cmd_numx2;
	speed_data[7] = ((cmd_data & 0xFF000000) >> 24);
	speed_data[8] = ((cmd_data & 0xFF0000) >> 16);
	speed_data[9] = ((cmd_data & 0xFF00) >> 8);
	speed_data[10] = (cmd_data & 0xFF);

	unsigned short crc = calc_crc(speed_data, 11);
	for (int i = 0; i < 11; i++)
		cmd_crc[i] = speed_data[i];
	cmd_crc[11] = (crc & 0xFF);
	cmd_crc[12] = ((crc & 0xFF00) >> 8);

	ser.write(cmd_crc, 13);
	usleep(C35*10);

	if(ID !=0x00){
		size_t now_read = ser.available();
		std::string read_buffer = ser.read(ser.available());
		// std::string read_buffer = ser.read(8);
		const unsigned char* receive_data = reinterpret_cast<const unsigned char*>(read_buffer.data()); // datatype change
		if (now_read == 8) {  //normal response
			unsigned char receive_data_nocrc[6];
			bool same = true;
			for (int i = 0; i < 6; i++) {
				receive_data_nocrc[i] = receive_data[i];
				same &= (receive_data_nocrc[i] == speed_data[i]);
			}
			if (!same) {
				ROS_INFO("receive data not matched!");
			}
			else {
				unsigned short crc_response = calc_crc(receive_data_nocrc, 6);
				same &= (receive_data[6] == (crc_response & 0xFF));
				same &= (receive_data[7] == ((crc_response & 0xFF00) >> 8));
				if (!same) {
					ROS_INFO("receive data crc wrong!");
				}
				ROS_INFO("send success!");
			}
		}
		else if (now_read == 5) {  //exception response
			unsigned char receive_data_nocrc[3];
			for (int i = 0; i < 3; i++) {
				receive_data_nocrc[i] = receive_data[i];
			}
			ROS_INFO("target ID: %d, FC: %X, error_code: %X.", receive_data[0], receive_data[1]-0x80, receive_data[2]);
		}
		else if (now_read == 0) {  //no reponse
			ROS_INFO("NO RESPONSE!");
		}
		else {
			
		}
	}
	else usleep(1500);
	usleep(C35);
};

void willy_base::send_cmd_single(unsigned char ID, unsigned short cmd_pos_, unsigned short cmd_data) 
{
	unsigned char speed_data[6], cmd_crc[8];
	speed_data[0] = ID;
	speed_data[1] = 0x06;
	speed_data[2] = ((cmd_pos_ & 0xFF00) >> 8);
	speed_data[3] = (cmd_pos_ & 0xFF);
	speed_data[4] = ((cmd_data & 0xFF00) >> 8);
	speed_data[5] = (cmd_data & 0xFF);

	unsigned short crc = calc_crc(speed_data, 6);
	for (int i = 0; i < 6; i++)
		cmd_crc[i] = speed_data[i];
	cmd_crc[6] = (crc & 0xFF);
	cmd_crc[7] = ((crc & 0xFF00) >> 8);

	ser.write(cmd_crc, 8);
	usleep(C35*10);

	if(ID !=0x00){
		size_t now_read = ser.available();
		std::string read_buffer = ser.read(ser.available());
		const unsigned char* receive_data = reinterpret_cast<const unsigned char*>(read_buffer.data()); // datatype change
		if (now_read == 8) {  //normal response
			unsigned char receive_data_nocrc[6];
			bool same = true;
			for (int i = 0; i < 6; i++) {
				receive_data_nocrc[i] = receive_data[i];
				same &= (receive_data_nocrc[i] == speed_data[i]);
			}
			if (!same) {
				//ROS_INFO("receive data not matched!");
			}
			else {
				unsigned short crc_response = calc_crc(receive_data_nocrc, 6);
				same &= (receive_data[6] == (crc_response & 0xFF));
				same &= (receive_data[7] == ((crc_response & 0xFF00) >> 8));
				if (!same) {
					//ROS_INFO("receive data crc wrong!");
				}
				//ROS_INFO("send success!");
			}
		}
		else if (now_read == 5) {  //exception response
			unsigned char receive_data_nocrc[3];
			for (int i = 0; i < 3; i++) {
				receive_data_nocrc[i] = receive_data[i];
			}
			ROS_INFO("target ID: %d, FC: %X, error_code: %X.", receive_data[0], receive_data[1]-0x80, receive_data[2]);
		}
		else if (now_read == 0) {  //no reponse
			//ROS_INFO("NO RESPONSE!");
		}
		else {
			
		}
	}
	else usleep(1500);
	usleep(C35);
};

int willy_base::read_cmd(unsigned char ID, unsigned short cmd_startpos_, unsigned short num) 
{
	unsigned char ask_data[6], cmd_crc[8];
	ask_data[0] = ID;
	ask_data[1] = 0x03;
	ask_data[2] = ((cmd_startpos_ & 0xFF00) >> 8);
	ask_data[3] = (cmd_startpos_ & 0xFF);
	ask_data[4] = ((num & 0xFF00) >> 8);
	ask_data[5] = (num & 0xFF);

	unsigned short crc = calc_crc(ask_data, 6);
	for (int i = 0; i < 6; i++)
		cmd_crc[i] = ask_data[i];
	cmd_crc[6] = (crc & 0xFF);
	cmd_crc[7] = ((crc & 0xFF00) >> 8);

	ser.write(cmd_crc, 8);
	usleep(C35*10);

	if(ID !=0x00){
		size_t now_read = ser.available();
		std::string read_buffer = ser.read(ser.available());
		const unsigned char* receive_data = reinterpret_cast<const unsigned char*>(read_buffer.data()); // datatype change
		if (now_read == (5+2*num)) {  //normal response
			int len = 3+2*num;
			unsigned char receive_data_nocrc[len];
			for (int i = 0; i < len; i++) {
				receive_data_nocrc[i] = receive_data[i];
			}
			unsigned short receive_message[num];
			if(receive_data_nocrc[2] != num*2){
				//ROS_INFO("NUMBER OF RESPONSE MISMATCHED!");
			}
			else {
				for(int i = 0; i<num;i++){
					receive_message[i] = ((receive_data[3+2*i]<<8)+(receive_data[4+2*i]));
				}
				unsigned short crc_response = calc_crc(receive_data_nocrc, len);
				if ((receive_data[len] != (crc_response & 0xFF)) || (receive_data[len+1] != ((crc_response & 0xFF00) >> 8))) {
					//ROS_INFO("receive data crc wrong!");
				}
				if(num == 2){
					return ((int)(receive_message[0] << 16) +(int)receive_message[1]);
					// if (ID == ID_1){
					// 	vr_vel.data = (int)(receive_message[0] << 16) +(int)receive_message[1];
					// }
					// else if (ID == ID_2){
					// 	vl_vel.data = (int)(receive_message[0] << 16) +(int)receive_message[1];
					// }
				}
				else if (num == 1){
					
				}
			}
			return 0;
		}
		else if (now_read == 5) {  //exception response
			unsigned char receive_data_nocrc[3];
			for (int i = 0; i < 3; i++) {
				receive_data_nocrc[i] = receive_data[i];
			}
			ROS_INFO("ID: %d, FC: %X, error_code: %X.", receive_data[0], receive_data[1]-0x80, receive_data[2]);
		}
		else if (now_read == 0) {  //no reponse
			//ROS_INFO("NO RESPONSE!");
		}
		else {
			
		}
	}
	else usleep(1500);
	return 0;
	usleep(C35);
};

void willy_base::run()
{
    ros::Rate loop_rate(hz);
	ros::Time Curr_T;
	std::string read_buffer;
	size_t now_read = 0;
	unsigned char R_En[4], L_En[4];
    willy_v2::Float_Header pub_VR, pub_VL;
	vr_vel.data = 0;
	vl_vel.data = 0;
    int vr = 0, vl = 0;
	unsigned int a = 100;
	vel_data_1 = 0;
	vel_data_2 = 0;
	//bool calibration_mode = true;
	//bool calibration_done = true;
	
	if(!is_motor_set){
		ROS_INFO("Motor Initializing...");
		send_cmd(0x00, 0x1200, 0);
		send_cmd(0x00, 0x138A, 0);
		send_cmd(ID_2, 0x0384, 1);
		send_cmd(0x00, 0x102A, 1);//Load holding function
		send_cmd(0x00, 0x0294, 1);//Impact softening filter
		//send_cmd(0x00, 0x0E54, 2);//Decelaration mode selection

		send_cmd(0x00 , 0x1030 , -1); // load holding torque limit
		send_cmd(0x00 ,0x0E3A,1500 ); //speed gain
		send_cmd(0x00 ,0x0E3E,150 ); //torque gain
		send_cmd(0x00, 0x0704, 200);  //torque limiting value(%)
		//send_cmd(0x00, 0x104A, 900);
		send_cmd(ID_1, 0x018C, 1);
		ROS_INFO("Configuring...");
		usleep(2000000);
		send_cmd(ID_2, 0x018C, 1);
		ROS_INFO("Configuring...");
		usleep(2000000);
		send_cmd(0x00, 0x0192, 1);
		ROS_INFO("Done.");
		ROS_INFO("Please restart motor to confirm changes!!");
		char confirm = 'n';
		while (confirm != 'y') {
			ROS_INFO("restarted? ([y] for yes): ");
			scanf("%c", &confirm);
		}
		is_motor_set = true;
		usleep(1500000);
	}


    while(ros::ok()) 
	{
	if (last_cmd_counter != cmd_counter) {
			VR = linear_vel + angular_vel * Wheel_Base;
			VL = linear_vel - angular_vel  * Wheel_Base;
			vr = (int)(VR * Gear_Ratio * rad2rev / (sec2min * Wheel_Radius));
			vl = (int)(VL * Gear_Ratio * rad2rev / (sec2min * Wheel_Radius));
			// if ((vr<0)&&(vl>0)){vr=0;}
			// else if ((vr>0)&&(vl<0)){vl=0;}
			calculate_cmd(vr, vl);
			}
			send_cmd(ID_1, 0x0484, vel_data_1);
			send_cmd(ID_2, 0x0484, vel_data_2);
			vr_vel.data = read_cmd(ID_1, 0x00CE, 2);
			vl_vel.data = read_cmd(ID_2, 0x00CE, 2);
			vr_temp.data=read_cmd(ID_1, 0x00F8,2);
			vl_temp.data=read_cmd(ID_2, 0x00F8,2);
			vr_cmd.data=read_cmd(ID_1, 0x00C8,2);
			vl_cmd.data=read_cmd(ID_2, 0x00C8,2);
			vr_load.data=read_cmd(ID_1, 0x00D8,2);
			vl_load.data=read_cmd(ID_2, 0x00D8,2);
			
			Curr_T = ros::Time::now();
			pub_VR.header.stamp = Curr_T;
			pub_VL.header.stamp = Curr_T;
			pub_VR.num = vr;
			pub_VL.num = vl;

			R_vel_pub.publish(pub_VR);
			L_vel_pub.publish(pub_VL);
			motor_vl_pub.publish(vl_vel);
			motor_vr_pub.publish(vr_vel);
			vr_temp_pub.publish(vr_temp);
			vl_temp_pub.publish(vl_temp);
			vr_cmd_pub.publish(vr_cmd);
			vl_cmd_pub.publish(vl_cmd);
			vr_load_pub.publish(vr_load);
			vl_load_pub.publish(vl_load);
		
			last_cmd_counter = cmd_counter;

			ros::spinOnce();
			loop_rate.sleep();	
		
    }
}

int willy_base::Serial_set(int Baudrate, std::string Port, int Parity) 
{
	try {
			ser.setPort(Port);
			ser.setBaudrate(Baudrate);
			serial::Timeout to;
			if(is_motor_set) to = serial::Timeout::simpleTimeout(1000);
			else to = serial::Timeout::simpleTimeout(1000);
			ser.setTimeout(to);
			serial::parity_t PARITY;
			if (Parity == 2) PARITY = serial::parity_t::parity_even;
			else if (Parity == 1) PARITY = serial::parity_t::parity_odd;
			else PARITY = serial::parity_t::parity_none;
			ser.setParity(PARITY);
			ser.open();
			return 1;
		}
	catch(serial::IOException& e) {
			return 0;
		}
};

void willy_base::callback(const geometry_msgs::Twist::ConstPtr &cmd) 
{
	angular_vel = cmd->angular.z;
	linear_vel = cmd->linear.x;

    cmd_counter += 1;
}

void willy_base::calculate_cmd(int vr,int vl)
{
	if (vl > 40) {
		if (vl >= 80 && vl <= 4000) vel_data_2 = vl;
		else if (vl > 4000) vel_data_2 = 4000;
		else vel_data_2 = 80;
		if (last_vl_dir != 1) send_cmd_single(ID_2, 0x007D, 0b0000000000111010);
		last_vl_dir = 1;
	}
	else if (vl < 40 && vl >-40) {
		vel_data_2 = 0;
		if (last_vl_dir != 0) send_cmd_single(ID_2, 0x007D, 0b0000000000110010);
		last_vl_dir = 0;
	}
	else {
		if (vl <= -80 && vl >= -4000) vel_data_2 = -vl;
		else if (vl < -4000) vel_data_2 = 4000;
		else vel_data_2 = 80;
		if (last_vl_dir != -1) send_cmd_single(ID_2, 0x007D, 0b0000000000011010);
		last_vl_dir = -1;
	}

	if (vr > 40) {
		if (vr >= 80 && vr <= 4000) vel_data_1 = vr;
		else if (vr > 4000) vel_data_1 = 4000;
		else vel_data_1 = 80;
		if (last_vr_dir != 1) send_cmd_single(ID_1, 0x007D, 0b0000000000111010);
		last_vr_dir = 1;
	}
	else if (vr < 40 && vr >-40) {
		vel_data_1 = 0;
		if (last_vr_dir != 0) send_cmd_single(ID_1, 0x007D, 0b0000000000110010);
		last_vr_dir = 0;
	}
	else {
		if (vr <= -80 && vr >= -4000) vel_data_1 = -vr;
		else if (vr < -4000) vel_data_1 = 4000;
		else vel_data_1 = 80;
		if (last_vr_dir != -1) send_cmd_single(ID_1, 0x007D, 0b0000000000011010);
		last_vr_dir = -1;
	}
}
