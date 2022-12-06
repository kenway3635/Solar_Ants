#! /usr/bin/python
# -*- coding:utf-8 -*-



from unittest.result import failfast
import cv2
import numpy as np
import time
import math
import rospy
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
from std_msgs.msg import String


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
        self.pub = rospy.Publisher('CameraCondition', String, queue_size=1)
        try:
            self.listener()
        except: raise BaseException("ROS Subscriber Error")
    def listener(self):rospy.Subscriber("image",Image,self.img_callback)     
    def img_callback(self,data): 
        self.raw_image=self.bridge.imgmsg_to_cv2(data,desired_encoding="passthrough")
    
    def line_detect(self, minlineLength = 60 , maxlineGap = 50):
        self.use_image = self.raw_image.copy()
        img = self.raw_image.copy()
        self.use_image = cv2.cvtColor(self.use_image,cv2.COLOR_BGR2GRAY)
        self.use_image = cv2.GaussianBlur(self.use_image,(3,3),sigmaX=1)
        self.use_image = cv2.Canny(self.use_image,150,225,apertureSize = 3 ,L2gradient= True) 
        self.use_image = cv2.dilate(self.use_image,self.kernel_dilate,iterations=2)
        self.use_image = cv2.erode(self.use_image,self.kernel_erode, iterations=1)
        
        linePoint = cv2.HoughLinesP(self.use_image,1,np.pi/180 , 
                                    50,None,minlineLength,maxlineGap)

        try:
            # find the longest line in this frame 
            Line_set = np.resize(linePoint,(linePoint.shape[0] ,4 ))
            

            for x1,y1,x2,y2  in Line_set:
            
                if abs(y2-y1) >  abs(x2-x1) : 
                    if math.sqrt((x2-x1)**2 + (y2-y1)**2) > 100:
                        cv2.line(img, (x1,y1),(x2,y2),(0,255,0),5)
                else : 
                    if math.sqrt((x2-x1)**2 + (y2-y1)**2) > 100:
                        cv2.line(img, (x1,y1),(x2,y2),(255,0,0),5)

        except: 
            pass
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    def CameraFail(self):
        print("fail")
        self.pub.publish("fail")

if __name__ == "__main__":

    fail_time = 0
    max_val = 0
    rospy.init_node("anomaly_detect" , anonymous=True)
    RosImage = ROS_image()
    RosImage.use_image = np.zeros((RosImage.height,RosImage.width,3) , dtype=np.uint8)
    
    RosImage.line_detect()
    time.sleep(1)
    img1 = RosImage.line_detect()
    blur1 = cv2.GaussianBlur(img1,(5,5),0)
    while  not rospy.is_shutdown():
        img2 = RosImage.line_detect()
        blur2 = cv2.GaussianBlur(img2,(5,5),0)
        blur1 = cv2.resize(blur1,(320,180))
        blur2 = cv2.resize(blur2,(320,180))
        #print(blur1.shape , blur2.shape)
        result = cv2.absdiff(blur1, blur2)

        #cv2.imshow("1",blur1)
        #cv2.imshow("2",blur2)
        cv2.imshow("diffShow",result)

        
        reduce_matrix = np.full((result.shape[0], result.shape[1]), 128)

        shift_value = result - reduce_matrix
        shift_sum = sum(map(sum, shift_value))
        diff = shift_sum / result.size + 128#-128~128

        #print(diff)

        if diff > 70:
            RosImage.CameraFail()
            fail_time+=1
            print(fail_time)
        if diff > max_val:
            print(max_val)
            max_val = diff
        
        img1 = img2.copy()
        blur1 = blur2.copy()

        if cv2.waitKey(100) & 0xFF == ord("q"): 
            break
    cv2.destroyAllWindows()