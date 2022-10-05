#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
import sys
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy
from model import CNN_ODOM_Model, Visual_IMU_Model, Model_0718
from to_npy import DataCapture

import torch 
import torchvision.transforms as T
import torch.nn as nn


if __name__ == '__main__':
  DC=DataCapture()
  DC.listener()
  DC.publisher()
  cmd=Twist()
  time = 0 #time stamp
  device = 'cuda' if torch.cuda.is_available() else 'cpu'
  model = Model_0718(input_dim=172810).to(device)
  save_path = './models/0718.ckpt'
  model.load_state_dict(torch.load(save_path,map_location='cpu'))
  model.eval()
  batch_size = 1
  data=np.zeros((batch_size,172810))
  #rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    frame=DC.raw_image
    #cv2.imshow('frame', frame)
    #cv2.waitKey(1)
    DC.Image_to_1D(frame)
    frame_1d=np.append(DC.image,DC.imu)
    
    for i in range(batch_size-1):
      data[i]=data[i+1]
    data[batch_size-1]=frame_1d
  
    #print (frame_1d)
    #print("image = ",len(DC.image),"odom  = ",len(DC.odom),"vel_input",len(DC.vel_input))
    input = torch.FloatTensor(data).to(device)
    predict = model(input)
    
    time=time+1
    if time >=batch_size:
      vel=predict[batch_size-1].cpu().detach().numpy()
      print ('estimated value is Linear: ',cmd.linear.x,'Angular :',cmd.angular.z)
      cmd.linear.x = vel[0]
      cmd.angular.z = vel[1]
      DC.pub.publish(cmd)
      #print('loss = ',(frame_1d[49155]-vel[0])+(frame_1d[49156]-vel[1]))
    #rate.sleep()
    '''
    vel=predict.cpu().detach().numpy()
    print('value =', vel)
    cmd.linear.x = vel[0]
    cmd.angular.z = vel[1]
    DC.pub.publish(cmd)
    '''
    #print('loss = ',(frame_1d[49155]-vel[0])+(frame_1d[49156]-vel[1]))
    
    
    #cv2.imshow("Image window", DC.raw_image)
    if cv2.waitKey(1)&0xFF == ord('q'):
      break
    

  #rospy.spin()
  #cv2.destroyAllWindows()