#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String

def f(x):
    # any operation
    pass

cap = cv2.VideoCapture(0)

cv2.namedWindow("Trackbars")
cv2.createTrackbar("L-H", "Trackbars", 142, 180, f)
cv2.createTrackbar("L-S", "Trackbars", 0, 255, f)
cv2.createTrackbar("L-V", "Trackbars", 90, 255, f)
cv2.createTrackbar("U-H", "Trackbars", 180, 180, f)
cv2.createTrackbar("U-S", "Trackbars", 246, 255, f)
cv2.createTrackbar("U-V", "Trackbars", 255, 255, f)

font = cv2.FONT_HERSHEY_COMPLEX

def talker():

    rospy.init_node('arrow', anonymous=True)
    pub = rospy.Publisher('/arrow', String, queue_size=1)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        l_h = cv2.getTrackbarPos("L-H", "Trackbars")
        l_s = cv2.getTrackbarPos("L-S", "Trackbars")
        l_v = cv2.getTrackbarPos("L-V", "Trackbars")
        u_h = cv2.getTrackbarPos("U-H", "Trackbars")
        u_s = cv2.getTrackbarPos("U-S", "Trackbars")
        u_v = cv2.getTrackbarPos("U-V", "Trackbars")

        lower_red = np.array([l_h, l_s, l_v])
        upper_red = np.array([u_h, u_s, u_v])

        mask = cv2.inRange(hsv, lower_red, upper_red)
        kernel = np.ones((4, 4), np.uint8)
        mask = cv2.erode(mask, kernel)

        # Contours detection
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
            x = approx.ravel()[0]
            y = approx.ravel()[1]

            if area > 10000:

                if len(approx) == 7:
                    cv2.putText(frame, "tip", (x, y), font, 1, (0, 0, 0))
                    cv2.drawContours(frame, [approx], 0, (0, 0, 0), 5)

            n = approx.ravel()

            i = 0
            if area > 400:
                print(approx)
                if len(approx) == 7:
                    acord=[]
                    cord=[]
                    sumx=0
                    sumy=0
                    for j in n :

                        if(i % 2 == 0):
                            x = n[i]
                            y = n[i + 1]
                            # String containing the co-ordinates.
                            string = str(x) + " " + str(y) 
                            
                            if(i == 0):
                                # text on pointed co-ordinate.
                                cv2.putText(frame, string, (x, y), font, 0.5, (255, 255, 0))
                                cord=[(x,y)]
                                #print(cord)
                                x1=x
                                y1=y
                            else:
                                # text on remaining co-ordinates.
                                cv2.putText(frame, string, (x, y), font, 0.5, (0, 255, 0))
                                acord=acord+[(x,y)]


                            if len(acord)==6:
                                #print(acord)
                                sumx=sumx+acord[0][0]+acord[1][0]+acord[2][0]+acord[3][0]+acord[4][0]+acord[5][0]
                                sumy=sumy+acord[0][1]+acord[1][1]+acord[2][1]+acord[3][1]+acord[4][1]+acord[5][1]
                                #print("Average of remainign coordinates=", str(sumx/4), ",", str(sumy/4))
                                x2=sumx/6
                                y2=sumy/6
                                
                            #print(int(slope))
                            #cv2.putText(frame, str(int(slope)), (x, y), font, 1, (0, 0, 0))
                        i = i + 1
                    ans=str(int(np.degrees(np.arctan((y2-y1)/(x2-x1)))))
                    print(ans)
                    cv2.putText(frame, ans, (10, 50), font, 1.5, (0,0,0))            


        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)

        key = cv2.waitKey(1)
        if (key == 27):
            break

        if rospy.is_shutdown():
            cap.release()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
