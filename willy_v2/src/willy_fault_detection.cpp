#include <willy_fault_detection.h>


using namespace willy_v2;
using namespace message_filters;

ros::Publisher cmd_vel_pub;

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "willy_fault_detection");
    ros::NodeHandle n;
	

    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 20);

    message_filters::Subscriber<Float_Header> RVC_sub(n,"/Right_Vel_Cmd", 1);
    message_filters::Subscriber<Float_Header> RVR_sub(n,"/Right_Vel_Response", 1);    
    message_filters::Subscriber<Float_Header> LVC_sub(n,"/Left_Vel_Cmd", 1);
    message_filters::Subscriber<Float_Header> LVR_sub(n,"/Left_Vel_Response", 1);
  
    typedef sync_policies::ApproximateTime<Float_Header, Float_Header, Float_Header, Float_Header> MySyncPolicy;

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), RVC_sub, RVR_sub, LVC_sub, LVR_sub);

    sync.registerCallback(boost::bind(&Right_detection, _1, _2, _3, _4));  

  
  /*
TimeSynchronizer<Float_Header, Float_Header> Right_sync(RVC_sub, RVR_sub, 20);
    TimeSynchronizer<Float_Header, Float_Header> Left_sync(LVC_sub, LVR_sub, 20);
    
    Right_sync.registerCallback(boost::bind(&Right_detection, _1, _2));
    Left_sync.registerCallback(boost::bind(&Left_detection, _1, _2)); 
*/
    ROS_INFO("START");
    ros::spin();
    return 0;
}


void Right_detection(const Float_Header::ConstPtr& msg_Right_cmd, const Float_Header::ConstPtr& msg_Right_rsp, const Float_Header::ConstPtr& msg_Left_cmd, const Float_Header::ConstPtr& msg_Left_rsp)
{
    if ((msg_Right_cmd->num!=0.0 && msg_Right_rsp->num==0.0)||(msg_Left_cmd->num!=0.0 && msg_Left_rsp->num==0.0))
        count += 1;
    else
        count = 0;

    ROS_INFO("count is %d\n",count);
    ROS_INFO("cmd is %f\n",msg_Right_cmd->num);
    ROS_INFO("rep is %f\n",msg_Right_rsp->num);
    

    if (count>=10) 
    {
        vel_msg.linear.x=0;
        vel_msg.linear.y=0;
        vel_msg.linear.z=0;
        vel_msg.angular.x=0;
        vel_msg.angular.y=0;
        vel_msg.angular.z=0;
        cmd_vel_pub.publish(vel_msg);
        count = 0;
    }
}

