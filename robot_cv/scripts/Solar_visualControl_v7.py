#! /usr/bin/python
# -*- coding:utf-8 -*-


import cv2 , datetime, rospy ,time,math   
import numpy as np 
from geometry_msgs.msg import Twist , Pose2D 
from std_msgs.msg import String,Int64, Bool,Float32  
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
from collections import namedtuple 
import argparse , time  
import threading 

# ---------- ROS Image subscriber ------------
class ROS_image(): 
    def __init__(self): 
    
        self.bridge = CvBridge() 
        self.height ,self.width = 180 ,320
        #self.margin = int(0.1*self.width)
        self.kernel_dilate = np.ones((3,3),np.uint8)
        self.kernel_erode = np.ones( (3,3),np.uint8)
        #self.sobel_kernel = np.array([[-1,2,-1] , [-1,2,-1] ,[-1,2,-1] ] ,dtype=np.int8 ) 
            
        self.raw_image = np.zeros((180,320,3) , dtype=np.uint8)
        self.use_image = None
        self.store_image = None

        self.cameraFail = 0

        try:
            self.listener()
        except: raise BaseException("ROS Subscriber Error")
        
    def listener(self):rospy.Subscriber("image",Image,self.img_callback)     
    def img_callback(self,data): self.raw_image=self.bridge.imgmsg_to_cv2(data,desired_encoding="passthrough")


    def preProcessing(self):     
        self.use_image = cv2.cvtColor(self.use_image,cv2.COLOR_BGR2GRAY)
        #cv2.imshow("GRAY",self.use_image)
        self.use_image = cv2.GaussianBlur(self.use_image,(3,3),sigmaX=1) 
        #self.use_image = cv2.filter2D(self.use_image,-1,self.sobel_kernel,delta=0)
        self.use_image = cv2.Canny(self.use_image,150,225,apertureSize = 3 ,L2gradient= True) 
        self.use_image = cv2.dilate(self.use_image,self.kernel_dilate,iterations=2)
        self.use_image = cv2.erode(self.use_image , self.kernel_erode , iterations=1)
        #self.use_image = cv2.morphologyEx(self.use_image,cv2.MORPH_CLOSE,self.kernel,iterations=1 )
        
    def line_detect(self,minlineLength = 60 , maxlineGap = 50, inUturn = False): 
        #self.use_image = cv2.bitwise_not(self.use_image)
        cv2.imshow("processed",self.use_image)
        linePoint = cv2.HoughLinesP(self.use_image,1,np.pi/180 , 
                                    50,None,minlineLength,maxlineGap)
        view = self.raw_image.copy()
        try: 
            # find the longest line in this frame 
            Line_set = np.resize(linePoint,(linePoint.shape[0] ,4 )) 
            
            angleList = []
            
            if not inUturn:
                for x1,y1,x2,y2  in Line_set:
                
                    if abs(y2-y1) >  abs(x2-x1) : 
                        if math.sqrt((x2-x1)**2 + (y2-y1)**2) > 100: 
                            cv2.line(view , (x1,y1),(x2,y2),(0,255,0),5) 
                            cal_angle= math.atan2(x2-x1,abs(y2-y1)+0.001) * 57.3
                            if y2-y1<0:
                                cal_angle = cal_angle*(-1)
                            angleList.append(cal_angle)
                if len(angleList):
                    self.cameraFail = 0
                    angle = sum(angleList)/len(angleList) 
                    detectable = True 
                else:
                    angle = 0
                    self.cameraFail += 1
                    detectable = False

            else:
                for x1,y1,x2,y2  in Line_set:
                
                    if abs(y2-y1) >  abs(x2-x1) : 
                        if math.sqrt((x2-x1)**2 + (y2-y1)**2) > 100: 
                            cv2.line(view , (x1,y1),(x2,y2),(0,255,0),5) 
                            cal_angle = abs(math.atan2(x2-x1,abs(y2-y1)+0.001)) * 57.3
                            angleList.append(cal_angle)
                    else : 
                        if math.sqrt((x2-x1)**2 + (y2-y1)**2) > 100: 
                            cv2.line(view , (x1,y1),(x2,y2),(0,255,255),5) 
                            cal_angle = 90 - abs(math.atan2(x2-x1,abs(y2-y1)+0.001)) * 57.3
                            angleList.append(cal_angle)
                if len(angleList) > 3:
                    angle_avg = sum(angleList) / len(angleList)
                    i=0
                    while i != len(angleList):
                        if abs(angleList[i] - angle_avg) > 5:
                            angleList.pop(i)
                            i-=1
                        i+=1
                if len(angleList) < 3:
                    self.cameraFail += 1
                else:
                    self.cameraFail = 0


                angle = 0 if not len(angleList)  else sum(angleList)/len(angleList)   
                detectable = True if angle else False
                
                aq.enqueue(angle)
        
        except: 
            angle = 0
            detectable = False
        
        finally:
            if not inUturn:
                acc_angle = angle
            elif self.cameraFail == 0:
                acc_angle = aq.get()
            else:
                acc_angle = 0
            
        
        return detectable , acc_angle , view
    
    def CameraFailReset(self):
        self.store_image = self.raw_image.copy()
        self.store_image = cv2.resize(self.store_image,(320,180))
        self.store_image = cv2.cvtColor(self.store_image, cv2.COLOR_BGR2GRAY)
        self.store_image = cv2.GaussianBlur(self.store_image,(3,3),sigmaX=1)
        self.cameraFail = 0


    def CameraFailDetect(self):
        img = self.raw_image.copy()
        img = cv2.resize(img,(320,180))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.GaussianBlur(img,(3,3),sigmaX=1)
        result = cv2.absdiff(self.store_image, img)
        cv2.imshow("diffShow",result)
        reduce_matrix = np.full((result.shape[0], result.shape[1]), 128)

        shift_value = result - reduce_matrix
        shift_sum = sum(map(sum, shift_value))
        diff = shift_sum / result.size + 128

        if diff > 70:
            self.cameraFail = -1
        else:
            if self.cameraFail == -1:
                print("Camera Fail be Solved")
                self.cameraFail = 0
            self.store_image = img.copy()

class anglequeue : 
    def __init__(self):
        self.size = 3
        self.q = [0]
    def enqueue(self,val):
        if -15 <(abs(val) - self.get()) < 15 : self.q.append(val)
        else: 
            print(abs(val) , self.get(),self.q)
            #input()
        if len(self.q) > self.size : self.q.pop(0) 
    def get(self) : 
        angle = weight = 0
        for i in self.q:
            weight+=1
            angle+= abs(i) * weight
        return angle / (self.size**2 + self.size) * 2

#----------- other Subscriber callback function 

class Robot(): 
    def __init__(self,enhance_factor=1): 
        self.visual_sw = False 
        self.flag = 0
        self.reverse = 1
        self.inUturn = False 
        self.UturnState = 1
        self.side = 5
        self.IR_left,self.IR_Right = None ,None 
        self.enhance_factor = enhance_factor
        #self.IMU = namedtuple("IMU",["x","y","theta"])(None,None,None)
        self.State = namedtuple("State",["Fall","Line","Angle"])(False,False,0)
        self.velocity = Twist()
        self.Register()
        
    def Register(self): 
        self.vehPub = rospy.Publisher("visual_cmd_vel",Twist,queue_size=1)
        self.linePub = rospy.Publisher("/line",Bool,queue_size=1)
        self.switchSub = rospy.Subscriber("/visualSW",Bool,self.switch_callback) 
        #self.poseSub = rospy.Subscriber("/pose2d",Pose2D,self.pose_callback,queue_size=1) 
        self.FallSub = rospy.Subscriber("/front_detect",Bool,self.front_callback)
        rospy.loginfo("Register Done ! ")

    def newVelocity(self,x,z,reverse=1): 
        
        self.velocity.linear.x = x * self.enhance_factor * reverse
        self.velocity.angular.z = z * self.enhance_factor * reverse
        self.vehPub.publish(self.velocity)
        
    def switch_callback(self,msg): self.visual_sw = msg.data 
    def front_callback(self,msg): self.State = self.State._replace(Fall=msg.data) 
    #def pose_callback(self,msg): self.IMU = self.IMU._replace(x = msg.x,y=msg.y,theta=msg.theta) 
        

    def Move(self): 
        print("------",self.State.Angle) 
        if not self.State.Fall and not self.State.Line :  
            self.newVelocity(0.3,0)
        elif self.State.Fall : 
            rospy.loginfo("Stop ! ")
            self.inUturn = True
            rospy.loginfo("#########  Start Uturn !!!! #########")
        elif not self.State.Fall and self.State.Line: 
            if abs(self.State.Angle) <= 5 : 
                self.newVelocity(0.3,0)
            else:
                if self.State.Angle > 0 :  
                    print("left turn !")   
                else: print("right turn !")
            
                self.newVelocity(0.12,-0.1) if self.State.Angle <0 else self.newVelocity(0.12,0.1) 


    def Uturn(self):
        rospy.loginfo(f"Uturn in {self.UturnState} state")
        print(self.State.Angle)
        if self.State.Angle == 0 :
            self.newVelocity(0,0.05,self.reverse)
            return

        if self.UturnState != 4:
            self.newVelocity(0,0.3,self.reverse)

        if self.UturnState == 6:
            if abs(self.State.Angle) < 10:
                self.inUturn = False
                self.UturnState = 1
                self.newVelocity(0,0)
                rospy.loginfo(" Utrun complete ! ")
                self.flag = self.flag+1 if self.visual_sw else 0
                self.reverse = (lambda flag : 1 if flag%2 == 0 else -1 )(self.flag)
                self.side = 5
        elif self.UturnState == 5:
            if self.State.Angle > 20 :
                self.UturnState = 6
        elif self.UturnState == 4:
            self.newVelocity(0.15,0)
            self.side -= 1
            rospy.loginfo(f"move: {self.side}")
            if self.side == 0:
                self.UturnState = 5
        elif self.UturnState == 3:
            if self.State.Angle < 10:
                self.UturnState = 4
        elif self.UturnState == 2:
            if self.State.Angle < 25:
                self.UturnState = 3
        else:
            if self.State.Angle > 30:
                self.UturnState = 2
        
    
 


if __name__ == "__main__": 
    parser = argparse.ArgumentParser() 
    parser.add_argument("--speed",help="modify the speed factor")
    
    rospy.init_node("visualControl" , anonymous=True)

    SolarAnt = Robot(enhance_factor=float(parser.parse_args().speed) if parser.parse_args().speed else 1)
    RosImage =ROS_image() 
    aq = anglequeue()
    time.sleep(1)
    RosImage.CameraFailReset()
    
    RosImage.use_image = np.zeros((RosImage.height,RosImage.width,3) , dtype=np.uint8)
    view = RosImage.use_image.copy()
    while not rospy.is_shutdown(): 
            
        if RosImage.raw_image.any() == True:
            RosImage.CameraFailDetect()
            RosImage.use_image = RosImage.raw_image.copy()

            RosImage.preProcessing() 
            line_detectable , line_angle, view  = RosImage.line_detect(inUturn = SolarAnt.inUturn)
            SolarAnt.State = SolarAnt.State._replace(Line=line_detectable,Angle=line_angle)
            SolarAnt.linePub.publish(SolarAnt.State.Line)
            #print(f"line_detect {line_detectable} , line angle {line_angle}")

            if RosImage.cameraFail == 3:
                SolarAnt.newVelocity(0,0)
                print("Camera Fail(Lines are incorrect)")
                time.sleep(1)

            elif RosImage.cameraFail == -1:
                SolarAnt.newVelocity(0,0)
                print("Camera Fail(FOD)")
                time.sleep(1)


            elif SolarAnt.visual_sw:
                if SolarAnt.inUturn:
                    SolarAnt.Uturn()
                else:
                    SolarAnt.Move()

            if not SolarAnt.visual_sw:
                print("manual mode")
                SolarAnt.inUturn = False
                SolarAnt.UturnState = 1
                SolarAnt.newVelocity(0,0)
                SolarAnt.flag = 0
                SolarAnt.reverse = 1
                SolarAnt.side = 5
                RosImage.CameraFailReset()
                time.sleep(1)

            rospy.loginfo(SolarAnt.State)
            
        cv2.imshow("draw",view)
        if cv2.waitKey(50) & 0xFF == ord("q"): 
            break 
    cv2.destroyAllWindows() 
    
cv2.destroyAllWindows()
