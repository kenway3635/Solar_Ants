import cv2 


cap = cv2.VideoCapture(0) 
fourcc = cv2.VideoWriter_fourcc(*"mp4v")
out = cv2.VideoWriter("test.avi",fourcc,20,(640,360))



assert cap.isOpened() , " Cap error "

while cap.isOpened() :
    ret,frame = cap.read() 
    frame = cv2.resize(frame,(640,360))
    out.write(frame) 
    cv2.imshow("test",frame)
    
    if cv2.waitKey(1) == ord("q"): break 
    
cap.release() 
out.release()
cv2.destroyAllWindows() 
        