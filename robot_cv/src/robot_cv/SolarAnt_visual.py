import cv2
import numpy as np
import math

# Initialize
#cap = cv2.imread("/home/croso1024/python_code/CV/sample_img/panelC14.jpg")
def init():
    cap = cv2.imread("/home/croso1024/python_code/CV/sample_img/panelC14.jpg")
    global kernel , height,width,margin ,IMG
    kernel = np.ones((7,7),np.uint8)
    height = 360
    width=480
    margin = int(0.15*width)
    IMG = cv2.resize(cap,(width,height))


def preProcessing(image,height,width) :
    #img = cv2.resize(image,(width,height))
    img = image
    img_back = img[:,:margin]
    img_front = img[:,-margin:]
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray,(5,5),0)
    #closing = cv2.morphologyEx(gray,cv2.MORPH_CLOSE,kernel,iterations=3)
    #ret,thresh = cv2.threshold(gray,0,255,cv2.THRESH_OTSU)
    ret,thresh = cv2.threshold(gray,0,255,cv2.THRESH_OTSU)
    thresh = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel,iterations=2)
    Back = thresh[:,:margin]
    Front= thresh[:,-margin:]
    return img_back,Back,img_front,Front

# def classification(image):
    
#     Back = image[:,:margin]
#     Front= image[:,-margin:]

#     return Front,Back

def corner(edge):
    edge = np.reshape(edge,(edge.shape[0],edge.shape[2])) #--> reshpae to x * 2
    #找出最小值（最上方）的元素所對應的索引們
    up_mean = 0 # 最上方元素的平均位置
    down_mean = 0 # 最底下元素的平均位置
    up_bound = np.min(edge[:,1])
    down_bound = np.max(edge[:,1])
    index_min = np.where(edge[:,1] == up_bound)[0] #找出所有最小值發生的索引
    index_max = np.where(edge[:,1] == down_bound)[0]
    #由所有最小（大）值的索引, 將它帶入來取得我們所要的"該索引對應的x值"
    for i in index_min: 
        up_mean += edge[i][0]
    up_mean = int(up_mean / len(index_min))
    for j in index_max:
        down_mean += edge[j][0]   
    down_mean = int(down_mean / len(index_max))

    return up_bound,up_mean,down_bound,down_mean


def calAngle(up_bound,up_mean,down_bound,down_mean):
    bound_diff = int(abs(down_bound-up_bound))
    diff = int((up_mean - down_mean ))
    ang = math.atan2( diff*2 , bound_diff) *57.3
    return ang

def detect(img,binary):
    detectable,angle = False ,0
    contour,hie1 = cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    
    for c in contour:
        M = cv2.moments(c)
        cX = int(M["m10"]/(M["m00"]+0.001))
        cY = int(M["m01"]/(M["m00"]+0.001))
        area = cv2.contourArea(c)
        if area>=20000:
            detectable = True
            print(c.shape)
            up_bound,up_mean,down_bound,down_mean = corner(c) 
            angle = calAngle(up_bound,up_mean,down_bound,down_mean)
            print(angle)
            #print(area,cX,cY)
            cv2.circle(img,(up_mean,up_bound),10,(0,0,255),-1)
            cv2.circle(img,(down_mean,down_bound),10,(0,0,255),-1)
    
    cv2.drawContours(img,contour,-1,(0,255,0),2)
    return detectable , angle
init()
if __name__ =="__main__":
    #init()
    image_back,binary_back,image_front,binary_front = preProcessing(IMG,height,width)

    front_detect , front_angle = detect(image_front,binary_front)
    back_detect,back_angle = detect(image_back,binary_back)
    State = [front_detect,back_detect,front_angle,back_angle]
    print(State)
    #contour,hie = cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    #contour,hie = cv2.findContours(image_c,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cv2.imshow("ww",IMG)
    cv2.imshow("frame_front",image_front)
    cv2.imshow("frame_back",image_back)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
