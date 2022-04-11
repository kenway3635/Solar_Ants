#! /usr/bin/python
import cv2
import numpy as np
import math
import time
#global IMG
def init():
    #cap = cv2.imread("/home/croso1024/python_code/CV/sample_img/panelCr0.jpg")
    global kernel , height,width,margin ,IMG,kernel_area,kernel_line
    kernel_area = np.ones((7,7),np.uint8)
    kernel_line = np.ones((3,3),np.uint8)
    height = 360
    width=480
    margin = int(0.1*width)
    #IMG = cv2.resize(cap,(width,height))
def preProcessing(image,height,width):
    #img = cv2.resize(image,(width,height))
    img = image
    #img_back = img
    #img_front = img
    img_back = img[:,:margin]
    img_front = img[:,-margin:]
    return img_back,img_front 
def area_detect(image,maxThresh=558000):
    
    detectable = False
    gray_scale = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    gray_scale = cv2.GaussianBlur(gray_scale,(3,3),0)
    ret,thresh =cv2.threshold(gray_scale,0,255,cv2.THRESH_OTSU)
    binary = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel_area,iterations=1)
    #ret,contour , hie = cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    ret,contour , hie = cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
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
def line_detect(image,minlineLength=130,maxlineGap=18):
    length,angle = 0,0
    detectable = False
    edgePoint = []
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray,(3,3),0)
    edges = cv2.Canny(image,180,220,apertureSize=3)
    edges = cv2.morphologyEx(edges,cv2.MORPH_CLOSE,kernel_line,iterations=2)
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
        angle = math.atan2( x2-x1 , abs(y2-y1)) *57.3
        detectable = True
        
    except:
        #print("linepoint no detect")
        pass
    
    return detectable , angle
# dist = np.array( [ -3.3489097336487306e-01, 1.2821145903063155e-01,
#        8.8711583787014583e-04, -1.2552852052560579e-03,
#        -2.3352766344466324e-02 ])
# mtx = np.array([ 3.0301263272855510e+02, 0., 3.0541949445278198e+02, 0.,
#        3.0253583660053522e+02, 2.3129592501103573e+02, 0., 0., 1. ])
# mtx = np.resize(mtx,(3,3))
init()
dist = np.array( [ -3.3489097336487306e-01, 1.2821145903063155e-01,
       8.8711583787014583e-04, -1.2552852052560579e-03,
       -2.3352766344466324e-02 ])
mtx = np.array([ 3.0301263272855510e+02, 0., 3.0541949445278198e+02, 0.,
       3.0253583660053522e+02, 2.3129592501103573e+02, 0., 0., 1. ])
mtx = np.resize(mtx,(3,3))
cap = cv2.VideoCapture(0)
assert cap.isOpened() ,"cap error"

while cap.isOpened():
    t_start = time.time()
    ret,frame = cap.read()

    frame = cv2.undistort(frame, mtx, dist, None, None)
    IMG = cv2.resize(frame,(width,height))
    image_back , image_front = preProcessing(IMG,height,width)
    detect_front=area_detect(image_front)
    
    detect_back,back_angle = line_detect(image_back)
    State = [detect_front,detect_back,back_angle]
    print(State)
    cv2.imshow("frame",IMG)
    t_end1 = time.time()
    if cv2.waitKey(200) &0xFF == ord("q"):
        print("end")
        break
    t_end2 = time.time()
    print(t_end1 - t_start)
    print(t_end2 - t_start)
cap.release()
cv2.destroyAllWindows()