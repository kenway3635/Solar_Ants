#! /usr/bin/python
# -*- coding:utf-8 -*-

from itertools import accumulate
import cv2 , datetime, rospy ,time,math   
import numpy as np 
from geometry_msgs.msg import Twist , Pose2D 
from std_msgs.msg import String,Int64, Bool,Float32  
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
from collections import namedtuple 
import argparse 
# ---------- ROS Image subscriber ------------
class ROS_image(): 
    def __init__(self): 
    
        self.bridge = CvBridge() 
        self.height ,self.width = 360 ,640
        self.margin = int(0.1*self.width) 
        self.kernel = np.ones( (3,3),np.uint8 )
        self.raw_image = np.zeros((180,320,3) , dtype=np.uint8)
        self.use_image = None
        try:self.listener()
        except: raise BaseException("ROS Subscriber Error")
        
    def listener(self):rospy.Subscriber("image",Image,self.img_callback)     
    def img_callback(self,data): self.raw_image=self.bridge.imgmsg_to_cv2(data,desired_encoding="passthrough")

    def preProcessing(self): 
        #self.use_image = self.raw_image.copy() 
        self.use_image =  cv2.cvtColor(self.use_image,cv2.COLOR_BGR2GRAY)  
        self.use_image = cv2.GaussianBlur(self.use_image,(3,3),1) 
        self.use_image = cv2.Canny(self.use_image,120,200,apertureSize = 3 ,L2gradient= True) 
        self.use_image = cv2.morphologyEx(self.use_image,cv2.MORPH_CLOSE,self.kernel,iterations=1 )
        
        #TODO 可能需要切掉部份區域
    def line_detect(self,minlineLength = 200 , maxlineGap = 18): 
      
        linePoint = cv2.HoughLinesP(self.use_image,1,np.pi/180 , 
                                    50,None,minlineLength,maxlineGap) 
        #print("test: linepoint",linePoint)
        try: 
            # find the longest line in this frame 
            count,accumulation_angle = 0 , 0
            for x1,y1,x2,y2  in np.resize(linePoint,(linePoint.shape[0],4)): 
                if abs(y2-y1) <  abs(x2-x1) : continue
                if (x2-x1)**2 + (y2-y1)**2 > 100: 
                    cv2.line(self.use_image,(x1,y1),(x2,y2),(0,255,0),5)
                    count+=1 
                    accumulation_angle += math.atan2(x2-x1,abs(y2-y1)+0.001) * 57.3 

            if count: 
                angle = accumulation_angle/count 
                detectable = True 
            else:
                angle = 0 
                detectable = False
        
        except: 
            angle = 0 
            detectable = False  
        return detectable , angle 
    
#----------- other Subscriber callback function 

class Robot(): 
    def __init__(self,enhance_factor=1): 
        self.visual_sw = False 
        self.flag = 0
        self.IR_left,self.IR_Right = None ,None 
        self.enhance_factor = enhance_factor
        self.IMU = namedtuple("IMU",["x","y","theta"])(None,None,None)
        self.State = namedtuple("State",["Fall","Line","Angle"])(False,False,0)
        self.velocity = Twist() 
        self.Register()
        
    def Register(self): 
        self.vehPub = rospy.Publisher("visual_cmd_vel",Twist,queue_size=1)
        self.linePub = rospy.Publisher("/line",Bool,queue_size=1)
        self.switchSub = rospy.Subscriber("/visualSW",Bool,self.switch_callback) 
        self.poseSub = rospy.Subscriber("/pose2d",Pose2D,self.pose_callback) 
        self.FallSub = rospy.Subscriber("/front_detect",Bool,self.front_callback)
        rospy.loginfo("Register Done ! ")

    def newVelocity(self,x,z,reverse=1): 
        
        self.velocity.linear.x = x * self.enhance_factor * reverse
        self.velocity.angular.z = z * self.enhance_factor * reverse
        self.vehPub.publish(self.velocity)
        
    def switch_callback(self,msg): self.visual_sw = not self.visual_sw 
    def front_callback(self,msg):self.State._replace(Fall=msg.data) 
    def pose_callback(self,msg): self.IMU._replace(x = msg.x,y=msg.y,theta=msg.theta) 
        
        

    def Move(self): 

        if not self.State.Fall and not self.State.Line :  
            self.newVelocity(0.3,0)
        elif self.State.Fall : 
            rospy.loginfo("Stop ! ")
            self.Uturn() 
        elif not self.State.Fall and self.State.Line: 
            if abs(self.State.Angle) <= 5 : 
                self.newVelocity(0.3,0)
            else:
                self.newVelocity(0.1,-0.06) if self.State.Angle <0 else self.newVelocity(0.1,0.06) 
        
        
    def Uturn(self): 
        rospy.loginfo("#########  Uturn !!!! #########")
        reverse = (lambda flag : 1 if flag%2 == 0 else -1 )(self.flag)
        
        if not self.IMU.x : raise BaseException("IMU Bug")
        
        # stage1 : turn 90 
        imu_temp = self.IMU.theta 
        while self.visual_sw: 
            self.newVelocity(0,0.15,reverse) 
            if 300 >= abs(self.IMU.theta - imu_temp ) >= 60 : break 
            rospy.loginfo(f"First turn angle displacement: { abs(self.IMU.theta - imu_temp )}")
        # stage2 : go forward 
        if self.visual_sw: 
            time.sleep(0.5) 
            imu_temp = self.IMU.y  
        while  self.visual_sw: 
            self.newVelocity(0.12,0) 
            if abs(imu_temp - self.IMU.y) >0.03: break
        # stage3 : turn 90 
        imu_temp = self.IMU.theta 
        while self.visual_sw: 
            self.newVelocity(0,0.15,reverse) 
            if 300 >= abs(self.IMU.theta - imu_temp ) >= 60 : break 
            rospy.loginfo(f"First turn angle displacement: { abs(self.IMU.theta - imu_temp )}")
        
        rospy.loginfo(" Utrun complete ! ")
        self.flag = self.flag+1 if self.visual_sw else 0 
        
        

if __name__ == "__main__": 
    parser = argparse.ArgumentParser() 
    parser.add_argument("--speed",help="modify the speed factor")
    
    rospy.init_node("visualControl" , anonymous=True)

    SolarAnt = Robot(enhance_factor=parser.parse_args().speed if parser.parse_args().speed else 1)
    RosImage =ROS_image() 
    
    while not rospy.is_shutdown(): 
        RosImage.use_image = np.zeros((RosImage.height,RosImage.width,3) , dtype=np.uint8) 
        
        if RosImage.raw_image.any() == True: 
            RosImage.use_image = RosImage.raw_image.copy() 
            
            RosImage.preProcessing() 
            line_detectable , line_angle  = RosImage.line_detect()
            SolarAnt.State._replace(Line=line_detectable , Angle=line_angle)
            
            if SolarAnt.visual_sw: SolarAnt.Move() 
            rospy.loginfo(SolarAnt.State)
        cv2.imshow("Frame",RosImage.use_image)
        if cv2.waitKey(100) & 0xFF == ord("q"): 
            break 
    cv2.destroyAllWindows() 
    
cv2.destroyAllWindows() 
    
    
