#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import sys
import numpy as np
import pandas as pd
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def start_node():
    rospy.init_node('image_pub')
    rospy.loginfo('image_pub node started')

cap = cv2.VideoCapture(0)
# 設定影像的尺寸大小
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 128)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 128)
#fps
cap.set(cv2.CAP_PROP_FPS , 24)
#set maxium buffer size
cap.set(cv2.CAP_PROP_BUFFERSIZE, 20)
start_node()
bridge = CvBridge()
pub = rospy.Publisher('image', Image, queue_size=10)
while (True):
  ret, frame = cap.read()
  cv2.imshow('frame', frame)
  imgMsg = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
  pub.publish(imgMsg)
  # 若按下 q 鍵則離開迴圈
  if cv2.waitKey(1) & 0xFF == ord('q'):
    break
  rospy.Rate(10).sleep()  # 10 Hz
cap.release()
cv2.destroyAllWindows()
rospy.spin()
