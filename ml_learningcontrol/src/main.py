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
from model import CNN_ODOM_Model

import torch 
import torchvision.transforms as T
import torch.nn as nn



class DataCapture():
  def __init__(self):
    self.vel_input=[]
    self.odom=[]
    self.image=[]
    self.bridge=CvBridge()
    self.raw_image= np.array((360,640,3), dtype=np.uint8)
    self.button = np.zeros(2)
    self.pub = None
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

  def imgmsg_to_cv2(self,img_msg):
    if img_msg.encoding != "bgr8":
        rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv

  def img_callback(self,data):
    self.raw_image=self.imgmsg_to_cv2(data)
    #cv2.imshow("Image window", self.raw_image)
    #cv2.waitKey(0)

  def listener(self):
    rospy.init_node('data_capture', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, self.vel_callback)
    rospy.Subscriber("pose2d", Pose2D, self.odom_callback)
    rospy.Subscriber("image", Image, self.img_callback)
    rospy.Subscriber("joy",Joy,self.joy_callback)

  def publisher(self):
    self.pub=rospy.Publisher('visual_cmd_vel', Twist, queue_size=10)
    


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
  DC.publisher()
  cmd=Twist()
  time = 0 #time stamp
  device = 'cuda' if torch.cuda.is_available() else 'cpu'
  model = CNN_ODOM_Model(input_dim=49155).to(device)
  save_path = 'CNN-odom_0525.ckpt'
  model.load_state_dict(torch.load(save_path,map_location='cpu'))
  model.eval()
  batch_size = 1
  data=np.zeros((batch_size,49155))
  #rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    frame=DC.raw_image
    #cv2.imshow('frame', frame)
    #cv2.waitKey(1)
    DC.Image_to_1D(frame)
    frame_1d=np.append(DC.image,DC.odom)
    
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
    
    
    cv2.imshow("Image window", DC.raw_image)
    if cv2.waitKey(1)&0xFF == ord('q'):
      break
    

  rospy.spin()
  #cv2.destroyAllWindows()