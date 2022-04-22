#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import pandas as pd
import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class DataCapture():
  def __init__(self):
    self.vel_input=[]
    self.odom=[]
    self.image=[]
    self.bridge=CvBridge()
    self.raw_image= np.array((128,128,3), dtype=np.uint8)

  def vel_callback(self,data):
    #rospy.loginfo(data.data)
    print('linear=%f,angular=%f',data.linear.x,data.angular.z)
    self.vel_input=[data.linear.x,data.angular.z]

  def odom_callback(self,data):
    #rospy.loginfo(data.data)
    print('x=%f,y=%f,theta=%f',data.x,data.y,data.theta)
    self.odom=[data.x,data.y,data.theta]
  
  def img_callback(self,data):
    try:
      self.raw_image=self.bridge.imgmsg_to_cv2(data,desired_encoding='passthrough')
    except CvBridgeError as e:
      print(e)


    #cv2.imshow("Image window", raw_image)
    #cv2.waitKey()

  def listener(self):
    rospy.init_node('data_capture', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, self.vel_callback)
    rospy.Subscriber("pose2d", Pose2D, self.odom_callback)
    rospy.Subscriber("image", Image, self.img_callback)


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
  while not rospy.is_shutdown():
    frame=DC.raw_image
    cv2.imshow('frame', frame)
    cv2.waitKey(3)
    DC.Image_to_1D(frame)
    #write array to csv file
    frame_1d=np.append(np.append(DC.image,DC.odom),DC.vel_input)
    print (frame_1d)
    pd.DataFrame(frame_1d).T.to_csv("data.csv", mode='a', header=False, index = False)
    # 若按下 q 鍵則離開迴圈
    if cv2.waitKey(1)&0xFF == ord('q'):
      break
  rospy.spin()
  cv2.destroyAllWindows()