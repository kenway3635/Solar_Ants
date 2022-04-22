#! /usr/bin/python
# -*- coding:utf-8 -*-

import cv2

import rospy
import numpy as np
import time,math 
from geometry_msgs.msg import Twist ,Pose2D
from std_msgs.msg import String,Int64,Bool,Float32
from robot_cv.SolarAnt_houph import area_detect,line_detect,preProcessing
from robot_cv.coefficient import load_coefficients
# Control Flow : 
"""
使用Houghs line版本!!!!
流程由suscribe接收到visual mode-> True開始
a. 1偵frame進入, 由preProcessing拆分出車身前後的部份以及處理後的binary
b. 喂入detect,由detect的return決定State -> 
            [前偵測(bool),後偵測(bool),前角度(float),後角度(float)]
c. 依據State不同進行操控策略
    c-1. State[0,0,x] --> 前後皆non-detect -->直走
    c-2. State[1,0,x] --> 前方偵測到 -->煞停 -->call Uturn
    c-3. State[0,1,x] --> 後方偵測到 -->準備出發
    檢查前方角度 ->夠小 -->出發 / -->過大 ->校正到小出發

由vel_state = [linear_x,angular_z,前次U轉方向] 保存速度相關狀態
"""
# 載入視覺控制參數 
#mtx, dist = load_coefficients('calibration_chessboard.yml')
dist = np.array( [ -3.3489097336487306e-01, 1.2821145903063155e-01,
       8.8711583787014583e-04, -1.2552852052560579e-03,
       -2.3352766344466324e-02 ])
mtx = np.array([ 3.0301263272855510e+02, 0., 3.0541949445278198e+02, 0.,
       3.0253583660053522e+02, 2.3129592501103573e+02, 0., 0., 1. ])
mtx = np.resize(mtx,(3,3))
visual_sw = True  # --> 初始化視覺工能的開關.
vel = Twist()
State = [False,False,0]  #-->初始化視覺state
Uturn_flag=int(0) # -->初始化轉向標記  偶數代表接下來要左U-turn,奇數為右Uturn
kernel = np.ones((7,7),np.uint8)
height = 360
width=480
margin = int(0.2*width)
#imu_theta ,imu_x,imu_y= 0,0,0


# 用來啟動程序的callback
def switch(msg):
    global visual_sw
    visual_sw = msg.data
    rospy.loginfo("Visual function:")

def angleRecoder(msg):
    global imu_x,imu_y ,imu_theta
    imu_x = msg.x
    imu_y = msg.y
    #if msg.theta>90 and msg.theta <=180: 
    if msg.theta <0:
        imu_theta= msg.theta + 360
    else:
        imu_theta = msg.theta
    # else:
    #     imu_theta =msg.theta
    #imu_theta = msg.theta 
    #print(msg.theta)
    #print(imuData)
def move(State):
    global vel ,Uturn_flag
    """
    由State決策車輛行動,vel_state儲存速度命令以及
    """
    if State[0] == 0 and State[1] == 0: #前後皆沒有偵測到物體-->直走
        
        vel.linear.x = 0.08
        vel.angular.z = 0 
        velPublisher.publish(vel)
        rospy.loginfo("Go forward")
        pass  # -->只要角度小於threshold ,持續上一動作
        #velPublisher.publish(vel)
    elif State[0] ==1 : #前方偵測到了
        vel.linear.x = 0
        vel.angular.z =0 
        rospy.loginfo(" Stop !! ")
        velPublisher.publish(vel)
        rospy.loginfo("Perform Uturn%2.1f",Uturn_flag)
        time.sleep(1)
        Uturn()

    elif State[0] ==0 and State[1] ==1: # 後方偵測到了
        #vel.linear.x =0
        if abs(State[2]) <= 7: # 當後方角度小於10度的時候可以繼續動
            
            vel.linear.x =0.08
            vel.angular.z = 0
            velPublisher.publish(vel)
        elif State[2] < -7:
            vel.linear.x =0.04
            vel.angular.z = 0.05
            velPublisher.publish(vel)
            rospy.loginfo("detect angle diff , compensation: Right turn")
        elif State[2] > 7:
            vel.linear.x =0.04
            vel.angular.z = -0.05
            velPublisher.publish(vel)
            rospy.loginfo("detect angle diff , compensation: Left turn")
        

def Uturn():  # --> uturn flag =0 left turn  /left turn --> postive angular z
    global cap,Uturn_flag
    rospy.loginfo("need Uturn !!!")
    rospy.loginfo("Uturn flag: %d" , Uturn_flag)
    flag = lambda Uturn_flag : 1 if Uturn_flag%2  == 0 else -1 
    flag = flag(Uturn_flag)
    cap.release()
    vel.linear.x =-0.05
    vel.angular.z = 0 
    velPublisher.publish(vel)
    time.sleep(0.5)
    vel.linear.x =0
    vel.angular.z = 0 
    velPublisher.publish(vel)
     # angle when call the Uturn funtion
    now_x,now_y,now_theta = imu_x,imu_y,imu_theta
    imu_goal = now_theta +flag* 90
    if imu_goal >=360:
        imu_goal -= 360
    rospy.loginfo("Imu now: %2.1f",imu_theta)
    rospy.loginfo("Imu Goal: %2.1f",imu_goal)
    while True:
        if flag*imu_theta > flag*imu_goal:
            break
        vel.linear.x = 0
        vel.angular.z = flag* 0.1
        velPublisher.publish(vel)
        rospy.loginfo("Imu now: %2.1f",imu_theta)
        rospy.loginfo("Imu Goal: %2.1f",imu_goal)
    vel.linear.x =0
    vel.angular.z = 0 
    velPublisher.publish(vel)
    vel.linear.x =0.05
    vel.angular.z = 0 
    velPublisher.publish(vel)
    time.sleep(1)
    vel.linear.x =0
    vel.angular.z = 0 
    velPublisher.publish(vel)
    now_x,now_y,now_theta = imu_x,imu_y,imu_theta
    imu_goal = now_theta + flag*90
    if imu_goal >=360:
        imu_goal -= 360
    rospy.loginfo("Imu now: %2.1f",imu_theta)
    rospy.loginfo("Imu Goal: %2.1f",imu_goal)
    while True:
        if flag*imu_theta > flag*imu_goal:
            break
        vel.linear.x = 0
        vel.angular.z = flag * 0.1
        velPublisher.publish(vel)
        rospy.loginfo("Imu now: %2.1f",imu_theta)
        rospy.loginfo("Imu Goal: %2.1f",imu_goal)
    rospy.loginfo("Uturn complete ! ,return to visual motion after 2 seconds")
    cap = cv2.VideoCapture(0)
    Uturn_flag +=1
    assert cap.isOpened() , "camera open error"

# 檢查相機啟動
cap = cv2.VideoCapture(0)
if not cap.isOpened(): 
    rospy.loginfo("camera error")
    raise BaseException("camera error")
# 初始化ros node
rospy.init_node("visualControl",anonymous=True)
velPublisher = rospy.Publisher("visual_cmd_vel",Twist,queue_size=2)
switchSubscribe = rospy.Subscriber("/visualSW",Bool,switch)
angleSubscribe =rospy.Subscriber("/pose2d",Pose2D,angleRecoder,queue_size=3)

#try:
rospy.loginfo("start")
rospy.loginfo(cap.isOpened())
while cap.isOpened(): 
    # Control flow start
    # rospy.loginfo("start2")
    ret,frame = cap.read()
    State = "visual function off "
    #print(mtx)
    frame = cv2.undistort(frame, mtx, dist, None, None)
    IMG = cv2.resize(frame,(width,height))
    #print(visual_sw)
    if visual_sw:
        image_back , image_front = preProcessing(IMG,height,width)
        detect_front=area_detect(image_front)
        detect_back,back_angle = line_detect(image_back)
        State = [detect_front,detect_back,back_angle]
        print(State)
        move(State)
    else:
        rospy.loginfo(State)
    cv2.imshow("frame",IMG)
    
    if cv2.waitKey(150) &0xFF == ord("q"):
        break
print("end")
#except:
#   rospy.loginfo("visual function error")
    
