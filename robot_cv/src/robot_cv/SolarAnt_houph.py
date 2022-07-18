#!/usr/bin/python
import cv2
import numpy as np
import math
#cap = cv2.imread("/home/croso1024/python_code/CV/sample_img/panelC5.jpg")
def init():
    #global IMG,height,width
    #cap = cv2.imread("/home/croso1024/python_code/CV/sample_img/panelC5.jpg")
    global kernel_area,kernel_line , height,width,margin ,IMG
    
    kernel_area = np.ones((7,7),np.uint8)
    kernel_line = np.ones((3,3),np.uint8)
    height = 360
    width=640
    margin = int(0.1*width) #--
    #IMG = cv2.resize(cap,(width,height))

def preProcessing(image,height,width) :
    #img = cv2.resize(image,(width,height))
    img = image
    #img_back = img[:,:int(margin*(1/3))]
    img_back = img[:,:2*margin]
    img_front = img[:,-margin:]
    #gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    #gray = cv2.GaussianBlur(gray,(5,5),0)
    #-------by momentum
    #ret,thresh = cv2.threshold(gray,0,255,cv2.THRESH_OTSU)
    #thresh = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel,iterations=2)
    #Back = thresh[:,:margin]
    #Front= thresh[:,-margin:]
    # ------by hough
    #edges = cv2.Canny(gray,50,150,apertureSize =3 )
    #thresh = cv2.HoughLinesP(edges,1,np.pi/180,15,None,100, 20)
    #Back = gray[:,:margin]
    #Front= gray[:,-margin:]
    return img_back,img_front 
    #return Back,Front
   

#for the brake 
def area_detect(image,maxThresh=12000):
    
    detectable = False
    gray_scale = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    gray_scale = cv2.GaussianBlur(gray_scale,(5,5),0)
    #ret,thresh =cv2.threshold(gray_scale,0,255,cv2.THRESH_OTSU)
    ret,thresh =cv2.threshold(gray_scale,60,255,cv2.THRESH_BINARY)
    binary = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel_area,iterations=2)
    contour , hie = cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    #cv2.imshow("area",binary)
    area = 0
    for c in contour:
        M = cv2.moments(c)
        cX = int(M["m10"]/(M["m00"]+0.001))
        cY = int(M["m01"]/(M["m00"]+0.001))
<<<<<<< HEAD
        area = max(area,cv2.contourArea(c))
=======
        area += cv2.contourArea(c) if cv2.contourArea(c) >500 else 0
        #area = max(area,cv2.contourArea(c))
>>>>>>> d6b70edb2fac14decdc2911aa57d9f1f632970aa
    print(area)
    if area>=maxThresh:
        detectable = True
    return detectable
def line_detect(image,minlineLength=130,maxlineGap=18):
    length,angle = 0,0
    detectable = False
    edgePoint = []
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray,(5,5),0)
    edges = cv2.Canny(image,120,200,apertureSize=3)
    edges = cv2.morphologyEx(edges,cv2.MORPH_CLOSE,kernel_line,iterations=2)
    
    #cv2.imshow("edge",edges)
    #if find something(line) go into try: or go to except
    linePoints = cv2.HoughLinesP(edges,1,np.pi/180,50,None,minlineLength,maxlineGap)
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
        #print("edge points =",edgePoint)

        cv2.line(image,(edgePoint[0],edgePoint[1]),(edgePoint[2],edgePoint[3]),(0,225,0),2)
        angle = math.atan2( x2-x1 , abs(y2-y1)+0.001) *57.3
        if y1>y2:
            angle = -angle

        detectable = True
        
    except:
        #print("linepoint no detect")
        pass
    
    return detectable , angle
    
init()#velPublisher.publish(vel)
        
if __name__ == "__main__":
    init()
    #back , front = preProcessing(IMG,height,width)
    #back_img , back_angle = line_detect(back)
#front_img,front_angle = line_detect(front)

# for x1,y1,x2,y2 in linee[0]:
#     cv2.line(IMG,(x1,y1),(x2,y2),(0,225,0),2)
# for i in range(linee.shape[0]):
#     x1,y1,x2,y2 = linee[i,0,0],linee[i,0,1],linee[i,0,2],linee[i,0,3]
#     cv2.line(IMG,(x1,y1),(x2,y2),(0,225,0),2)
    cv2.imshow("we",IMG)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
# if __name__ =="__main__":
#     init()
#     edge,linediagram = preProcessing(IMG,height,width)
#     cv2.imshow("edges",edge)
#     cv2.imshow("lien:",linediagram)

#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
