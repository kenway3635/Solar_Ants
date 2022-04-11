#!/usr/bin/python
import cv2
import numpy as np
import math
cap = cv2.VideoCapture(0)
assert cap.isOpened() ,"error"
while(cap.isOpened()):
    mtx = np.array([ [3.0301263272855510e+02, 0., 3.0541949445278198e+02], [0.,
       3.0253583660053522e+02, 2.3129592501103573e+02], [0., 0., 1.] ])
    dist =  np.array([ -3.3489097336487306e-01, 1.2821145903063155e-01,
       8.8711583787014583e-04, -1.2552852052560579e-03,
       -2.3352766344466324e-02 ])
    #print(mtx)
    ret, frame = cap.read()
    if ret==True:
        frame = cv2.flip(frame,0)
        dst = cv2.undistort(frame, mtx, dist, None, None)
        img_rgb = cv2.cvtColor(dst,cv2.COLOR_BGR2RGB)
        gray = cv2.cvtColor(dst,cv2.COLOR_RGB2GRAY)
        #_,thresh = cv2.threshold(gray, np.mean(gray), 255, cv2.THRESH_BINARY_INV)
        _,thresh = cv2.threshold(gray,0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        mask_gray = cv2.inRange(gray,0,70)
        kernel = np.ones((3,3), np.uint8)
        edges = cv2.dilate(cv2.Canny(thresh,200,255),kernel)
        #cv2.RETR_LIST
        cnt = sorted(cv2.findContours(edges, cv2.RETR_CCOMP , cv2.CHAIN_APPROX_NONE)[-2], key=cv2.contourArea)[-1]
        mask = np.zeros((480,640), np.uint8)
        mask_contour = cv2.drawContours(mask, [cnt],-1, 255, -1)
        mask_or = cv2.bitwise_or(mask_gray,mask_contour)
        mask_or = cv2.dilate(mask_or,None)
        dst_2 = cv2.bitwise_and(dst, dst, mask=mask_or)
        segmented = cv2.cvtColor(dst_2, cv2.COLOR_BGR2RGB)
        cv2.imshow('se',segmented)
        cv2.imshow('frame',img_rgb)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break
cap.release()
cv2.destroyAllWindows()

