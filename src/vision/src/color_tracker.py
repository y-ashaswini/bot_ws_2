#!/usr/bin/env python
import cv2
import numpy as np

# reference for hsv
# https://cvexplained.wordpress.com/2020/04/28/color-detection-hsv/

font = cv2.FONT_HERSHEY_SIMPLEX  

def nothing(x):
    pass


# def rgb_to_hsv(r, g, b):
#     r, g, b = r/255.0, g/255.0, b/255.0
#     mx = max(r, g, b)
#     mn = min(r, g, b)
#     df = mx-mn
#     if mx == mn:
#         h = 0
#     elif mx == r:
#         h = (60 * ((g-b)/df) + 360) % 360
#     elif mx == g:
#         h = (60 * ((b-r)/df) + 120) % 360
#     elif mx == b:
#         h = (60 * ((r-g)/df) + 240) % 360
#     if mx == 0:
#         s = 0
#     else:
#         s = (df/mx)*100
#     v = mx*100
#     return [h, s, v]



# Open the camera
cap = cv2.VideoCapture(0) 
 
# Create a window
cv2.namedWindow('frame')
 
# create trackbars for color change
# cv2.createTrackbar('lowH','image',0,179,nothing)
# cv2.createTrackbar('highH','image',179,179,nothing)
 
# cv2.createTrackbar('lowS','image',0,255,nothing)
# cv2.createTrackbar('highS','image',255,255,nothing)
 
# cv2.createTrackbar('lowV','image',0,255,nothing)
# cv2.createTrackbar('highV','image',255,255,nothing)

# for detecting red
lower_hsv = np.array([0, 50, 20]) 
higher_hsv = np.array([20, 255, 255])
 
while(True):
    ret, frame = cap.read()
 
    # get current positions of the trackbars
    # ilowH = cv2.getTrackbarPos('lowH', 'image')
    # ihighH = cv2.getTrackbarPos('highH', 'image')
    # ilowS = cv2.getTrackbarPos('lowS', 'image')
    # ihighS = cv2.getTrackbarPos('highS', 'image')
    # ilowV = cv2.getTrackbarPos('lowV', 'image')
    # ihighV = cv2.getTrackbarPos('highV', 'image')
    
    # convert color to hsv because it is easy to track colors in this color model
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # lower_hsv = np.array([ilowH, ilowS, ilowV])
    # higher_hsv = np.array([ihighH, ihighS, ihighV])


    # Apply the cv2.inrange method to create a mask
    mask = cv2.inRange(hsv, lower_hsv, higher_hsv)
    # Apply the mask on the image to extract the original color
    frame = cv2.bitwise_and(frame, frame, mask=mask)
    # cv2.imshow('image', frame)
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]


    center = None
    if(len(cnts)>0):
        c = max(cnts, key=cv2.contourArea)
        ((box_x, box_y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        X = int(box_x)
        Y = int(box_y)

        print('Target color object detected')
        print('X:%d'%X)  
        print('Y:%d'%Y)  
        print("---------")
        cv2.putText(frame,'Target Detected',(40,60), font, 0.5,(255,255,255),1,cv2.LINE_AA)
        cv2.rectangle(frame, (int(box_x-radius), int(box_y+radius)), (int(box_x+radius),int(box_y-radius)) ,(255,255,255),1)
    else:
        cv2.putText(frame,'Target Detecting',(40,60), font, 0.5,(255,255,255),1,cv2.LINE_AA)  
        print('No target color object detected') 

    cv2.imshow('frame',frame)


    # Press q to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()