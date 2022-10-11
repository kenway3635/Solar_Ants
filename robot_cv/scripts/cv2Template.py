#! /usr/bin/python
# -*- coding:utf-8 -*-

import cv2 ,rospy 
import numpy as np 
import time,math 


### Uturn 


### Move  


### DIP  #### 



#### init 


cap = cv2.VideoCapture(0)
if not cap.isOpened(): 
    print("camera error")
    raise BaseException("camera error")


print("start")

while cap.isOpened()  : 
    
    frame = cv2.resize(cap.read()[1] , (480,360))  
    frame4show = frame.clone()
    
    if visual_sw and Uturn  : 
        pass 
        ## frame -> DIP 
        ## move( state )
    
    
    cv2.imshow(frame4show)
    if cv2.waitKey(1) &0xFF == ord("q") :
        break 
    
print("end")

cap.release() 
cv2.destroyAllWindows() 
    
    