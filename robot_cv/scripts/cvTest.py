import cv2
print(cv2.__version__)
cap = cv2.VideoCapture(13)

assert cap.isOpened() ,"cap error"

while cap.isOpened():
    ret,frame = cap.read()
    cv2.imshow("frame",frame)

    if cv2.waitKey(1) &0xFF ==ord('q'):  
    	break
cap.release()
cv2.destroyAllWindows()




