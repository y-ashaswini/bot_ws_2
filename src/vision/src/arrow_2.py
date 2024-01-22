#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
import time

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
        img = cv2.GaussianBlur(img, (11,11), 0)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        corners = cv2.goodFeaturesToTrack(gray,7,0.01,10)
        corners = np.int0(corners)

        ct='a'
        for i in corners:
            x,y = i.ravel()
            print(x,y)
            cv2.circle(img,(x,y),3,(255,255,0),-1)
            cv2.putText(img, ct, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2, cv2.LINE_AA )
            ct=ct+'a'

        xmax, ymax = (np.max(corners, axis = 0)).ravel()
        xmin, ymin = (np.min(corners, axis = 0)).ravel() 

        
        if( abs(xmax-xmin) > abs(ymax-ymin)):
            if(np.count_nonzero(corners[:,0,0] == xmax) == 2):
                print('LEFT')
            else:
                print('RIGHT')
        else:
            if(np.count_nonzero(corners[:,0,1] == ymax) == 2):
                print('UP')
            else:
                print('DOWN')   

        cv2.imshow('image',img)
        time.sleep(2)

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
