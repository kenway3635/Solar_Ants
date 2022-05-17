#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy

class DataCapture():
  def __init__(self):
    self.vel_input=[]
    self.odom=[]
    self.image=[]
    self.bridge=CvBridge()
    self.raw_image= np.array((360,640,3), dtype=np.uint8)
    self.button = np.zeros(2)

  def vel_callback(self,data):
    #rospy.loginfo(data.data)
   #print('linear=%f,angular=%f',data.linear.x,data.angular.z)
    self.vel_input=[data.linear.x,data.angular.z]

  def joy_callback(self,data):
    self.button[0] = data.buttons[4]
    self.button[1]=data.buttons[5]

  def odom_callback(self,data):
    #rospy.loginfo(data.data)
    #print('x=%f,y=%f,theta=%f',data.x,data.y,data.theta)
    self.odom=[data.x,data.y,data.theta]
  
  def img_callback(self,data):
    self.raw_image=self.bridge.imgmsg_to_cv2(data,desired_encoding='passthrough')
    #cv2.imshow("Image window", self.raw_image)
    #cv2.waitKey(0)

  def listener(self):
    rospy.init_node('data_capture', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, self.vel_callback)
    rospy.Subscriber("pose2d", Pose2D, self.odom_callback)
    rospy.Subscriber("image", Image, self.img_callback)
    rospy.Subscriber("joy",Joy,self.joy_callback)

  def Image_to_1D(self,image):
    #image=cv2.resize(image,(128,128))
    arr=np.array(image)
    #print(arr.size)
    arr_1d=np.reshape(arr,-1)
    #np.append(arr_1d,'\n')
    #print(arr_1d)
    self.image=arr_1d

if __name__ == '__main__':
  DC=DataCapture()
  DC.listener()
  start = False
  i = 0 #time stamp
  total = None
  while not rospy.is_shutdown():
    frame=DC.raw_image
    #cv2.imshow('frame', frame)
    #cv2.waitKey(1)
    DC.Image_to_1D(frame)
    #write array to csv file
    frame_1d=np.append(np.append(DC.image,DC.odom),DC.vel_input)
    print("start recording = ",start,"time save = ",i)
    #print (frame_1d)
    print("image = ",len(DC.image),"odom  = ",len(DC.odom),"vel_input",len(DC.vel_input))
    if DC.button[0]:
        start = True
    if DC.button[1]:
        start = False
        np.save('train',total)
    if start:
      i=i+1
      if i==1 :
        total = frame_1d
      else:
        total = np.vstack((total,frame_1d))
    # 若按下 q 鍵則離開迴圈
  rospy.spin()
  cv2.destroyAllWindows()
