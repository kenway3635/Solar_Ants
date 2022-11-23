#!/usr/bin/python
# -*- coding: utf-8 -*-

from turtle import left
import cv2
import sys
import numpy as np
import sys
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import configparser

def start_node():
    rospy.init_node('ZED_image_pub')
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

def camera_Info(pos, res):
    res_list = {'vga':[640,480],'hd':[1280,720],'fhd':[1920,1080],'2k':[2560,1440]}
    info = CameraInfo()

    info.height = res_list[res][1]
    info.width = res_list[res][0]

    config = configparser.ConfigParser()
    config.read('./SN000011563.conf')
    s = (pos+"_CAM_"+res).upper()
    cam_cfg = config[s]
    
    info.distortion_model = 'pinhole'
    info.D = [float(cam_cfg['k1']),float(cam_cfg['k2']),float(cam_cfg['p1']),float(cam_cfg['p2'])]
    info.K = [float(cam_cfg['fx']),0,float(cam_cfg['cx']),
                        0,float(cam_cfg['fy']),float(cam_cfg['cy'])
                        ,0,0,1]

    stereo_cfg  = config['STEREO']
    RZ = float(stereo_cfg[('RZ_'+res).upper()])
    RX = float(stereo_cfg[('CV_'+res).upper()])
    RY = float(stereo_cfg[('RX_'+res).upper()])

    
    Rz, _ = cv2.Rodrigues(np.array([0, 0, RZ]))
    Ry, _ = cv2.Rodrigues(np.array([0, RY, 0]))
    Rx, _ = cv2.Rodrigues(np.array([RX, 0, 0]))
    R  = np.dot(Rz, np.dot(Ry, Rx)) # Rz*Ry*Rx
    print ("R = ",R)
    info.R = np.hstack((R[0],R[1],R[2]))
    if pos == 'left':
        info.P = [float(cam_cfg['fx']),0,float(cam_cfg['cx']),0,
                            0,float(cam_cfg['fy']),float(cam_cfg['cy']),0,
                            0,0,1,0]
    elif pos == 'right':
        info.P = [float(cam_cfg['fx']),0,float(cam_cfg['cx']),-float(stereo_cfg['Baseline'])*float(cam_cfg['fx']),
                    0,float(cam_cfg['fy']),float(cam_cfg['cy']),0,
                    0,0,1,0]
    return info

    
    
    

def main():
    cap = cv2.VideoCapture(2)
    # 設定影像的尺寸大小
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
    #fps
    cap.set(cv2.CAP_PROP_FPS , 10)
    #set maxium buffer size
    #cap.set(cv2.CAP_PROP_BUFFERSIZE, 20)
    #disable auto focus
    #cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    #cap.set(cv2.CAP_PROP_FOCUS, 42)
    #sety exposure time
    #cap.set(cv2.CAP_PROP_EXPOSURE,100)
    #set image bit channels
    start_node()
    bridge = CvBridge()

    left_pub = rospy.Publisher('ZED/left/image', Image, queue_size=1)
    right_pub = rospy.Publisher('ZED/right/image',Image,queue_size = 1)
    left_info_pub = rospy.Publisher('ZED/left/camera_info',CameraInfo,queue_size =1)
    right_info_pub = rospy.Publisher('ZED/right/camera_info',CameraInfo,queue_size = 1)
    left_info = camera_Info('left','vga')
    right_info = camera_Info('left','vga')
    while cap.isOpened() :
        ret, frame = cap.read()
        #frame=cv2.resize(frame,(320,180))
        height, width, channels = frame.shape   
        left_Image = frame[0:height, 0:int(width/2)] #this line crops
        right_Image = frame[0:height, int(width/2):width] #this line crops
        #cv2.imshow('left', left_Image)
        #cv2.imshow('right',right_Image)
        left_imgMsg = cv2_to_imgmsg(left_Image)
        right_imgMsg = cv2_to_imgmsg(right_Image)
 

        left_pub.publish(left_imgMsg)
        right_pub.publish(right_imgMsg)
        left_info_pub.publish(left_info)
        right_info_pub.publish(right_info)

    # 若按下 q 鍵則離開迴圈
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break
    rospy.Rate(10).sleep()  # 10 Hz
    #rospy.spin()


main()