#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
import math

    
cap = cv2.VideoCapture(0)

def talker():
    rospy.init_node('arrow', anonymous=True)
    # pub = rospy.Publisher('/arrow', String, queue_size=1)
    # rate = rospy.Rate(10)


    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break


        img = frame
        #convert the image to grayscale
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        #apply canny edge detection to the image
        edges = cv2.Canny(gray,50,150,apertureSize = 3)

        #show what the image looks like after the application of previous functions
        cv2.imshow("canny'd image", edges)

        # cv2.waitKey(0)

        # perform HoughLines on the image
        lines = cv2.HoughLines(edges,1,np.pi/180, 20)

        #create an array for each direction, where array[0] indicates one of the lines and array[1] indicates the other, which if both > 0 will tell us the orientation
        left = [0, 0]
        right = [0, 0]
        up = [0, 0]
        down = [0, 0]
      

        if lines is not None:
            for object in lines:
                rho = object[0][0]
                theta = object[0][1]
                # print(rho, theta)
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                # print(pt1, pt2)
                cv2.line(img, pt1, pt2, (0,0,255), 3, 2)


                # example object:
                # [[-110.           2.8448865]]

                # rho = object[0][0]
                # theta = object[0][1]

                #iterate through the lines that the houghlines function returned
                # cases for right/left arrows

                if ((np.round(theta, 2)) >= 1.0 and (np.round(theta, 2)) <= 1.1) or ((np.round(theta,2)) >= 2.0 and (np.round(theta,2)) <= 2.1):
                    if (rho >= 20 and rho <=  30):
                        left[0] += 1
                    elif (rho >= 60 and rho <= 65):
                        left[1] +=1
                    elif (rho >= -73 and rho <= -57):
                        right[0] +=1
                    elif (rho >=148 and rho <= 176):
                        right[1] +=1

                # cases for up/down arrows
                elif ((np.round(theta, 2)) >= 0.4 and (np.round(theta,2)) <= 0.6) or ((np.round(theta, 2)) >= 2.6 and (np.round(theta,2))<= 2.7):
                    if (rho >= -63 and rho <= -15):
                        up[0] += 1
                    elif (rho >= 67 and rho <= 74):
                        down[1] += 1
                        up[1] += 1
                    elif (rho >= 160 and rho <= 171):
                        down[0] += 1

        cv2.imshow("red lines", img)


        if left[0] >= 1 and left[1] >= 1:
            print("left")
        elif right[0] >= 1 and right[1] >= 1:
            print("right")
        elif up[0] >= 1 and up[1] >= 1:
            print("up")
        elif down[0] >= 1 and down[1] >= 1:
            print("down")
        else:
            print("can't detect direction")

        print(up, down, left, right)
        print()

        # press esc to exit
        key = cv2.waitKey(1)
        if (key == 27):
            break

        # time.sleep(2)

        if rospy.is_shutdown():
            cap.release()
            cv2.destroyAllWindows()



if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
