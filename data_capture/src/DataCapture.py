#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import sys
import numpy as np
import pandas as pd
import sys
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def start_node():
    rospy.init_node('image_pub')
    rospy.loginfo('image_pub node started')
def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tostring()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg

cap = cv2.VideoCapture(3)
# 設定影像的尺寸大小
cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,180)
#fps
#cap.set(cv2.CAP_PROP_FPS , 30)
#set maxium buffer size
#cap.set(cv2.CAP_PROP_BUFFERSIZE, 20)
start_node()
bridge = CvBridge()
pub = rospy.Publisher('image', Image, queue_size=5)
while (True):
  ret, frame = cap.read()
  frame=cv2.resize(frame,(128,128))
  cv2.imshow('frame', frame)
  imgMsg = cv2_to_imgmsg(frame)
  pub.publish(imgMsg)
  # 若按下 q 鍵則離開迴圈
  if cv2.waitKey(1) & 0xFF == ord('q'):
    break
  rospy.Rate(10).sleep()  # 10 Hz
cap.release()
cv2.destroyAllWindows()
rospy.spin()
