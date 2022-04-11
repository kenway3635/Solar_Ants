#! /usr/bin/python
# -*- coding:utf-8 -*-
import cv2
import rospy
import numpy as np
import time,math 
from geometry_msgs.msg import Twist 
from std_msgs.msg import String,Int64,Bool
#from  SolarAnt_visual import calAngle,corner,preProcessing,detect
#from SolarAnt_visual import detect, preProcessing
#from SolarAnt_visual import init,preProcessing,corner,calAngle,detect


# Control Flow : 
"""
使用形心版本!!!!
流程由suscribe接收到visual mode-> True開始
a. 1偵frame進入, 由preProcessing拆分出車身前後的部份以及處理後的binary
b. 喂入detect,由detect的return決定State -> 
            [前偵測(bool),後偵測(bool),前角度(float),後角度(float)]
c. 依據State不同進行操控策略
    c-1. State[0,0,x,x] --> 前後皆non-detect -->直走
    c-2. State[1,0,x,x] --> 前方偵測到 -->煞停 
    檢查前方角度 ->夠小 call Uturn / ->過大 ->校正到小call Uturn
    c-3. State[0,1,x,x] --> 後方偵測到 -->準備出發
    檢查前方角度 ->夠小 -->出發 / -->過大 ->校正到小出發

由vel_state = [linear_x,angular_z,前次U轉方向] 保存速度相關狀態
"""

def preProcessing(image,height,width) :
    #img = cv2.resize(image,(width,height))
    img = image
    img_back = img[:,:margin]
    img_front = img[:,-margin:]
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray,(5,5),0)
    edges = cv2.Canny(gray,50,150,apertureSize =3 )
    thresh = cv2.HoughLinesP(edges,1,np.pi/180,50,None,30,10)
    #closing = cv2.morphologyEx(gray,cv2.MORPH_CLOSE,kernel,iterations=3)
    #ret,thresh = cv2.threshold(gray,100,255,cv2.THRESH_BINARY_INV)
    #ret,thresh = cv2.threshold(gray,0,255,cv2.THRESH_OTSU)
    #thresh = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel,iterations=3)
    Back = thresh[:,:margin]
    Front= thresh[:,-margin:]
    return img_back,Back,img_front,Front


# 用來啟動程序的callback
def switch(msg):
    visual_sw = msg.data
    rospy.loginfo("Visual function:",visual_sw)

def move(State,Uturn_flag):
    global vel
    """
    由State決策車輛行動,vel_state儲存速度命令以及
    """
    if State[0] == 0 and State[1] == 0: #前後皆沒有偵測到物體-->直走
        vel.linear.x = 1
        vel.angular.z = 0 
        rospy.loginfo("Go forward")
        velPublisher.publish(vel)
    elif State[0] ==1 and State[1] ==0: #前方偵測到了
        vel.linear.x = 0
        vel.angular.z =0 
        rospy.loginfo(" Stop !! ")
        velPublisher.publish(vel)
        rospy.loginfo("Perform U turn task for ",Uturn_flag,"times")
        time.sleep(1)
        Uturn(Uturn_flag)
    elif State[0] ==0 and State[1] ==1: # 後方偵測到了
        vel.linear.x =0
        if State[3] >= 5: # 當後方角度小於5度的時候可以繼續動
            vel.angular.z =0.5
            velPublisher.publish(vel)
        elif State[3] <= -5:
            vel.angular.z = -0.5
            velPublisher.publish(vel)
            








def Uturn(Uturn_flag):
    global vel
    pass
# 載入視覺控制參數 
global visual_sw
#mtx, dist = load_coefficients('calibration_chessboard.yml')
#print(mtx)
mtx = np.array([[3.0301263272855510e+02, 0. ,3.0541949445278198e+02 ],[0. , 3.0253583660053522e+02, 2.3129592501103573e+02],[0. , 0. ,1]])
dist = np.array([ -3.3489097336487306e-01, 1.2821145903063155e-01,
       8.8711583787014583e-04, -1.2552852052560579e-03,
       -2.3352766344466324e-02 ])
visual_sw = True  # --> 初始化視覺工能的開關.
vel = Twist()
State = [False,False,0,0]  #-->初始化視覺state
Uturn_flag=0 # -->初始化轉向標記  偶數代表接下來要左U-turn,奇數為右Uturn

kernel = np.ones((7,7),np.uint8)
height = 360
width=480
margin = int(0.15*width)
# 檢查相機啟動
cap = cv2.VideoCapture(0)
if not cap.isOpened(): 
    rospy.loginfo("camera error")
    raise BaseException("camera error")
# 初始化ros node
rospy.init_node("visualControl",anonymous=True)
velPublisher = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
switchSubscribe = rospy.Subscriber("/visualSW",Bool,switch)
try:
    rospy.loginfo("start")
    rospy.loginfo(cap.isOpened())
    while cap.isOpened(): 
        # Control flow start
        # rospy.loginfo("start2")
        ret,frame = cap.read()
        frame = cv2.undistort(frame, mtx, dist, None, None)
        IMG = cv2.resize(frame,(width,height))
        if visual_sw:
            image_back,binary_back,image_front,binary_front = preProcessing(IMG,height,width)
            front_detect , front_angle = detect(image_front,binary_front)
            back_detect,back_angle = detect(image_back,binary_back)
            State = [front_detect,back_detect,front_angle,back_angle]
            pass
        rospy.loginfo(State)
        cv2.imshow("frame",IMG)
        cv2.imshow("back",binary_back)
        if cv2.waitKey(100) &0xFF == ord("q"):
            break
    print("end")
except:
    rospy.loginfo("visual function error")
    
