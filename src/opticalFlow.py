import cv2
import numpy as np
cap = cv2.VideoCapture("IMG_1634.mov")

ret, frame1 = cap.read()
prvs = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
hsv = np.zeros_like(frame1)
hsv[...,1] = 255

while(1):
    ret, frame2 = cap.read()
    frame = np.array(frame2[...,0])

    next = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)

    flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 15, 3, 5, 1.2, 0)
    uAV = (np.sum(flow[1:20,0])//len(flow[1:20,0]))//1920
    vAV = np.sum(flow[1:20,1]) // len(flow[1:20,1])//1920

    mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
    print(mag,ang)
    if not ret:
        break

    cv2.waitKey(ord('q'))
    cv2.imshow('test',frame2)

cap.release()
cv2.destroyAllWindows()
