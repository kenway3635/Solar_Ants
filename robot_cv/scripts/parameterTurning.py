#!/usr/bin/python 3

import cv2 
import numpy as np 
import math


###############  initize namedWindows ########
cv2.namedWindow("area_detect")
def test(value):

    pass
    
cv2.createTrackbar("detect_size","area_detect",0,100,test) 
cv2.setTrackbarPos("detect_size","area_detect",50)

cv2.createTrackbar("binary_lb","area_detect",0,255,test)
cv2.setTrackbarPos("binary_lb","area_detect",100)

cv2.createTrackbar("OPEN_iter","area_detect",1,5,test)
cv2.setTrackbarPos("OPEN_iter","area_detect",1)

cv2.createTrackbar("marginSize","area_detect",0,30,test)
cv2.setTrackbarPos("marginSize","area_detect",12)

cv2.namedWindow("line_detect") 
cv2.createTrackbar("minlineLength","line_detect",100,255,test)
cv2.setTrackbarPos("minlineLength","line_detect",200)


cv2.createTrackbar("maxlineGap","line_detect",10,50,test)
cv2.setTrackbarPos("maxlineGap","line_detect",18)


cv2.createTrackbar("canny_lb","line_detect",0,150,test) 
cv2.setTrackbarPos("canny_lb","line_detect",120)

cv2.createTrackbar("canny_ub","line_detect",150,255,test) 
cv2.setTrackbarPos("canny_ub","line_detect",200)

cv2.createTrackbar("CLOSE_iter","line_detect",1,5,test)
cv2.setTrackbarPos("CLOSE_iter","line_detect",1)



#cv2.namedWindow("otherParameter") 


###############  customizable parameter ########
############### Basic parameter ################
kernel_area = np.ones((7,7),np.uint8)
kernel_line = np.ones((3,3),np.uint8)
height = 360
width=480
#margin =  int(cv2.getTrackbarPos("marginSize","area_detect")*0.01 *width )
enhance_factor = 1 
def preProcessing(image,height,width):
    margin =  int(cv2.getTrackbarPos("marginSize","area_detect")*0.01 *width )
    #img = cv2.resize(image,(width,height))
    img = image
    #img_back = img[:,:int(margin*(1/3))]
    img_back = img[:,:margin]
    img_front = img[:,-margin:]
    return img_back,img_front 

def area_detect(image,maxThresh=558000):
    maxThresh = cv2.getTrackbarPos("detect_size","area_detect") *500
    binary_lb = cv2.getTrackbarPos("binary_lb","area_detect")
    OPEN_iter = cv2.getTrackbarPos("OPEN_iter","area_detect")
    
    detectable = False
    gray_scale = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    gray_scale = cv2.GaussianBlur(gray_scale,(3,3),0)
    #ret,thresh =cv2.threshold(gray_scale,0,255,cv2.THRESH_OTSU)
    ret,thresh =cv2.threshold(gray_scale,binary_lb,255,cv2.THRESH_BINARY)
    binary = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel_area,iterations=OPEN_iter)
    #ret,contour , hie = cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    contour,hie  = cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cv2.imshow("area",binary)
    for c in contour:
        M = cv2.moments(c)
        cX = int(M["m10"]/(M["m00"]+0.001))
        cY = int(M["m01"]/(M["m00"]+0.001))
        area = cv2.contourArea(c)
        #print(area)
        if area>=maxThresh:
            detectable = True
    return detectable


def line_detect(image,minlineLength=None,maxlineGap=None):
    minlineLength = cv2.getTrackbarPos("minlineLength","line_detect")
    maxlineGap = cv2.getTrackbarPos("maxlineGap","line_detect")
    canny_lb =  cv2.getTrackbarPos("canny_lb","line_detect")
    canny_ub = cv2.getTrackbarPos("canny_ub","line_detect")
    CLOSE_iter  = cv2.getTrackbarPos("CLOSE_iter","line_detect")
    
    length,angle = 0,0
    detectable = False
    edgePoint = []
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray,(7,7),sigmaX=1)
    #edges = cv2.Canny(image,120,200,apertureSize=3)
    edges = cv2.Canny(image,canny_lb,canny_ub,apertureSize=3)
    edges = cv2.morphologyEx(edges,cv2.MORPH_CLOSE,kernel_line,iterations=CLOSE_iter)
    cv2.imshow("edges",edges)
    #if find something(line) go into try: or go to except
    linePoints = cv2.HoughLinesP(edges,1,np.pi/180,80,None,minlineLength,maxlineGap)
    try:
        #print(linePoints)
        #print(type(linePoints),linePoints.shape)
        linePoints = np.resize(linePoints,(linePoints.shape[0],4))
        for i in range(linePoints.shape[0]):
            x1,y1,x2,y2 = linePoints[i]
            length_buffer = (x2-x1)**2 + (y2-y1)**2 
            if length_buffer > length:
                length = length_buffer
                edgePoint = [x1,y1,x2,y2]
        print(edgePoint)

        cv2.line(image,(edgePoint[0],edgePoint[1]),(edgePoint[2],edgePoint[3]),(0,225,0),2)
       # angle = math.atan2( x2-x1 , abs(y2-y1)) *57.3
        angle = math.atan2( y2-y1 , abs(x2-x1)) *57.3
        detectable = True
        
    except:
        #print("linepoint no detect")
        pass
    
    return detectable , angle

cap = cv2.VideoCapture(3)
assert cap.isOpened() ,"cap error"

while cap.isOpened() :

    ret,frame = cap.read()
    #frame = cv2.undistort(frame, mtx, dist, None, None)
    IMG = cv2.resize(frame,(width,height))
    image_back , image_front = preProcessing(IMG,height,width)
    detect_front=area_detect(image_front)
    detect_back,back_angle = line_detect(image_back)
    State = [detect_front,detect_back,back_angle]
    print(State)
    cv2.imshow("frame",IMG)
    if cv2.waitKey(200) &0xFF == ord("q"):
        print("end")
        break

cap.release()
cv2.destroyAllWindows()
