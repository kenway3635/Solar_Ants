import cv2
print(cv2.__version__)
cap = cv2.VideoCapture(3)
cap.set(cv2.CAP_PROP_POS_FRAMES,30)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,360)
assert cap.isOpened() ,"cap error"
i =0
while cap.isOpened():
    ret,frame = cap.read()
    frame = cv2.resize(frame,(640,360)) 
    i+=1         
    cv2.imshow("frame",frame)
    print(i)
    if cv2.waitKey(20) &0xFF ==ord('q'):  
    	break
cap.release()
cv2.destroyAllWindows()




