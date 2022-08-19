#!/usr/bin/python 3

import cv2 
import numpy as np 
import math


###############  initize namedWindows ########

def test(value):
    pass 

def kernel_change(value): 
    global kernel_line ,blurKernel_size
    if not value%2 == 0: 
        kernel_line = np.ones((value,value) ,np.uint8)
        blurKernel_size = value
   
        
cv2.namedWindow("line_detect") 
cv2.createTrackbar("minlineLength","line_detect",100,255,test)
cv2.setTrackbarPos("minlineLength","line_detect",200)


cv2.createTrackbar("maxlineGap","line_detect",10,50,test)
cv2.setTrackbarPos("maxlineGap","line_detect",18)


cv2.createTrackbar("canny_lb","line_detect",0,150,test) 
cv2.setTrackbarPos("canny_lb","line_detect",120)

cv2.createTrackbar("canny_ub","line_detect",150,255,test) 
cv2.setTrackbarPos("canny_ub","line_detect",200)

cv2.createTrackbar("Dilate_iter","line_detect",1,5,test)
cv2.setTrackbarPos("Dilate_iter","line_detect",1)

cv2.createTrackbar("Hough_coeifficient","line_detect",10,100,test) 
cv2.setTrackbarPos("Hough_coeifficient","line_detect",50)

cv2.createTrackbar("BlurKernel","line_detect",3,13,kernel_change) 
cv2.setTrackbarPos("BlurKernel","line_detect",5)

cv2.createTrackbar("Erode_iter","line_detect",1,5,test )
cv2.setTrackbarPos("Erode_iter","line_detect",1)
#cv2.namedWindow("otherParameter") 


###############  customizable parameter ########
############### Basic parameter ################

kernel_line = np.ones((3,3),np.uint8)
erode = np.ones((3,3),np.uint8)
blurKernel_size  = 3
height = 360
width=640
#margin =  int(cv2.getTrackbarPos("marginSize","area_detect")*0.01 *width )
enhance_factor = 1 

deNoise_kernel = np.array(
    [[5,5,5,5,5],[1,1,1,1,1],[1,1,1,1,1],[1,1,1,1,1],[5,5,5,5,5]]
    ,dtype =  np.int8 
)
deNoise_kernel = deNoise_kernel / deNoise_kernel.sum()

#print(deNoise_kernel)
def line_detect(image,minlineLength=None,maxlineGap=None):
    minlineLength = cv2.getTrackbarPos("minlineLength","line_detect")
    maxlineGap = cv2.getTrackbarPos("maxlineGap","line_detect")
    canny_lb =  cv2.getTrackbarPos("canny_lb","line_detect")
    canny_ub = cv2.getTrackbarPos("canny_ub","line_detect")
    Dilate_iter  = cv2.getTrackbarPos("Dilate_iter","line_detect")
    Erode_iter = cv2.getTrackbarPos("Erode_iter","line_detect")
    Hough = cv2.getTrackbarPos("Hough_coeifficient","line_detect") 
    
    length,angle = 0,0
    detectable = False
    edgePoint = []
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    # Filter \\\
    blur = cv2.GaussianBlur(gray,(blurKernel_size,blurKernel_size),sigmaX=1)
    #blur = cv2.medianBlur(gray,blurKernel_size)
    
    #blur = cv2.filter2D(gray,-1,deNoise_kernel , delta=-5)
    cv2.imshow("before Canny",blur)
    #ret ,blur = cv2.threshold(blur,127,255,cv2.THRESH_OTSU) 
    edges = cv2.Canny(blur,canny_lb,canny_ub,apertureSize=3,L2gradient = True)
    #edges = (255-edges)
    #edges = cv2.morphologyEx(edges,cv2.MORPH_CLOSE,kernel_line,iterations=CLOSE_iter)
    #edges = np.ones((360,640) ,dtype=np.uint8 )
    edges = cv2.dilate(edges,kernel_line,iterations=Dilate_iter)
    edges = cv2.erode(edges,erode,iterations=Erode_iter)
    
    print(edges.shape)
    cv2.imshow("edges",edges)
    #if find something(line) go into try: or go to except
    linePoints = cv2.HoughLinesP(edges,1,np.pi/180,Hough,None,minlineLength,maxlineGap)
    try:
        print("##########")
        #print(linePoints)
        #print(type(linePoints),linePoints.shape)
        linePoints = np.resize(linePoints,(linePoints.shape[0],4))
        #for i in range(linePoints.shape[0]):
        count =0
        added_angle = 0
        for x1,y1,x2,y2 in linePoints:  
           # x1,y1,x2,y2 = linePoints[i]
            if (abs(y2-y1)> abs(x2-x1))  :
                length_buffer = (x2-x1)**2 + (y2-y1)**2
                if length_buffer > 100:
                    cv2.line(image,(x1,y1),(x2,y2),(0,225,0),10)
                    added_angle += math.atan2( y2-y1 , abs(x2-x1)) *57.3
                    count +=1
        if count:
            angle = added_angle/count
            detectable = True
        else:
            angle = 0                    
        #print(edgePoint)
        print("DODODO")
        #cv2.line(image,(edgePoint[0],edgePoint[1]),(edgePoint[2],edgePoint[3]),(0,225,0),10)
        #angle = math.atan2( x2-x1 , abs(y2-y1)) *57.3
        #angle = math.atan2( y2-y1 , abs(x2-x1)) *57.3
        #detectable = True
        
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
    
    
    detect_back,back_angle = line_detect(IMG)
    State = [detect_back,back_angle]
    #print(State)
    cv2.imshow("frame",IMG)
    if cv2.waitKey(200) &0xFF == ord("q"):
        print("end")
        break

cap.release()
cv2.destroyAllWindows()
