#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


font = cv2.FONT_HERSHEY_SIMPLEX  

def nothing(x):
    pass


cap = cv2.VideoCapture(0) 
bridge = CvBridge()


# for detecting red
lower_hsv = np.array([0, 50, 20]) 
higher_hsv = np.array([20, 255, 255])
 

def talker():
    rospy.init_node('image', anonymous=True)
    pub = rospy.Publisher('/webcam', Image, queue_size=1)
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_hsv, higher_hsv)
        frame = cv2.bitwise_and(frame, frame, mask=mask)

        msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        pub.publish(msg)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if rospy.is_shutdown():
            cap.release()

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
