#! /usr/bin/python
# -*- coding:utf-8 -*-

import cv2
import datetime
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
Uturn_flag=0 # -->初始化轉向標記  偶數代表接下來要左U-turn,奇數為右Uturn
kernel = np.ones((7,7),np.uint8)
height = 360
width=480
margin = int(0.2*width)
imuData =0

# 用來啟動程序的callback
def switch(msg):
    global visual_sw
    visual_sw = msg.data
    rospy.loginfo("Visual function:")

def angleRecoder(msg):
    global imu_x,imu_y ,imu_theta
    imu_x = msg.x
    imu_y = msg.y
    imu_theta = msg.theta

def Uturn():
    global cap,Uturn_flag,visual_sw
    # preWork for freeze camera
    rospy.loginfo("need Uturn !!!")
    rospy.loginfo("Uturn flag: %d" , Uturn_flag)
    flag = lambda Uturn_flag : 1 if Uturn_flag%2  == 0 else -1 
    flag =flag(Uturn_flag)
    cap.release()
    # -------------------step1. backward
    if not  "imu_x" in globals():
        rospy.loginfo("imu fail !")
        visual_sw = False
        raise BaseException("IMU BUG")
    # while True:
    #     if abs(now_imu-imu_x) >0.1:
    #         break
    #     vel.linear.x = -flag*0.04
    #     vel.angular.z = 0 
    #     velPublisher.publish(vel)
    #----------------- step2. turn
     # angle when call the Uturn funtion
    now_imu = imu_theta
    while True:
        angleDisplacement = abs(imu_theta - now_imu)
        if angleDisplacement >= 85 and angleDisplacement <= 275:
            break
        vel.linear.x = 0
        vel.angular.z = flag * 0.2
        velPublisher.publish(vel)
        rospy.loginfo("angle_displacement: %2.1f",angleDisplacement)
        
    vel.linear.x =0
    vel.angular.z = 0 
    velPublisher.publish(vel)
     #------------------ step3. Slide
    now_imu = imu_y
    while True:
        if abs(now_imu - imu_y) >0.05:
            break
        vel.linear.x = 0.04
        vel.angular.z = 0 
        velPublisher.publish(vel)
    #----------------- step4. turn
    now_imu =imu_theta
    while True:
        angleDisplacement = abs(imu_theta - now_imu)
        if angleDisplacement >= 85 and angleDisplacement <= 275:
            break
        vel.linear.x = 0
        vel.angular.z = flag * 0.2
        velPublisher.publish(vel)
        rospy.loginfo("angle_displacement: %2.1f",angleDisplacement)
    rospy.loginfo("Uturn complete ! ,return to visual motion after 2 seconds")
    vel.linear.x =0
    vel.angular.z = 0 
    velPublisher.publish(vel)
    cap = cv2.VideoCapture(0)
    Uturn_flag +=1
    assert cap.isOpened() , "camera open error"

def Move(State):
    global cap,Uturn_flag
    flag = lambda Uturn_flag : 1 if Uturn_flag%2  == 0 else -1 
    flag =flag(Uturn_flag)
    if State[0] == 0 and State[1] == 0: #前後皆沒有偵測到物體-->直走
        vel.linear.x = 0.06
        vel.angular.z = 0 
        velPublisher.publish(vel)
        rospy.loginfo("Go forward")

    elif State[0] ==1 : #前方偵測到了
        vel.linear.x = 0
        vel.angular.z =0 
        rospy.loginfo(" Stop !! ")
        velPublisher.publish(vel)
        rospy.loginfo("Perform Uturn%2.1f",Uturn_flag)
        time.sleep(1)
        Uturn()
        
    elif State[0] ==0 and State[1] ==1: # 後方偵測到了
        if abs(State[2]) <= 5: # 當後方角度小於10度的時候可以繼續動
            
            vel.linear.x =0.08
            vel.angular.z = 0
            velPublisher.publish(vel)
        elif State[2] < -5:
            vel.linear.x =0.04
            vel.angular.z = -flag*0.05
            velPublisher.publish(vel)
            rospy.loginfo("detect angle diff , compensation: Right turn")
        elif State[2] > 5:
            vel.linear.x =0.04
            vel.angular.z = flag*0.05
            velPublisher.publish(vel)
            rospy.loginfo("detect angle diff , compensation: Left turn")

# 檢查相機啟動
cap = cv2.VideoCapture(0)
if not cap.isOpened(): 
    rospy.loginfo("camera error")
    raise BaseException("camera error")
# 初始化ros node
rospy.init_node("visualControl",anonymous=True)
velPublisher = rospy.Publisher("visual_cmd_vel",Twist,queue_size=1)
switchSubscribe = rospy.Subscriber("/visualSW",Bool,switch)
angleSubscribe =rospy.Subscriber("/pose2d",Pose2D,angleRecoder,queue_size=1)
#影像儲存: 
fourcc = cv2.VideoWriter_fourcc(*'XVID')
name=str(datetime.datetime.now())

output_img = cv2.VideoWriter("ww.avi",fourcc,15.0,(width,height))
#output_img = cv2.VideoWriter("aaa.avi",fourcc,15.0,(width,height))
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
        
        Move(State)
    else:
        rospy.loginfo(State)
    output_img.write(IMG)
    cv2.imshow("frame",IMG)
    
    if cv2.waitKey(20) &0xFF == ord("q"):
        break
print("end")
#except:
#   rospy.loginfo("visual function error")
    
cap.release()
output_img.release()
cv2.destroyAllWindows()