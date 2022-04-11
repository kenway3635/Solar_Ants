#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import pandas as pd
import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
class DataCapture():
  def __init__(self):
    self.vel_input=[]
    self.odom=[]
    self.image=[]
  def vel_callback(self,data):
    #rospy.loginfo(data.data)
    print('linear=%f,angular=%f',data.linear.x,data.angular.z)
    self.vel_input=[data.linear.x,data.angular.z]


  def odom_callback(self,data):
    #rospy.loginfo(data.data)
    print('x=%f,y=%f,theta=%f',data.x,data.y,data.theta)
    self.odom=[data.x,data.y,data.theta]
    
  def listener(self):
    rospy.init_node('data_capture', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, self.vel_callback)
    rospy.Subscriber("pose2d", Pose2D, self.odom_callback)


  def Image_to_1D(self,image):
    image=cv2.resize(image,(128,128))
    arr=np.array(image)
    #print(arr.size)
    arr_1d=np.reshape(arr,-1)
    #np.append(arr_1d,'\n')
    #print(arr_1d)
    self.image=arr_1d

if __name__ == '__main__':
  DC=DataCapture()
  DC.listener()
  cap = cv2.VideoCapture(0)
  while (True):
    ret, frame = cap.read()
    cv2.imshow('frame', frame)
    DC.Image_to_1D(frame)
    #write array to csv file
    frame_1d=np.append(np.append(DC.image,DC.odom),DC.vel_input)
    print (frame_1d)
    pd.DataFrame(frame_1d).T.to_csv("data.csv", mode='a', header=True, index = False)
    # 若按下 q 鍵則離開迴圈
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break
  cap.release()
  cv2.destroyAllWindows()
  rospy.spin()
