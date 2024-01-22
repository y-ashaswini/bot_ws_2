#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import PointCloud2, Image


def callbackfn(data):
    print(type(data))
    width = data.width # 640
    height = data.height # 480

    center = width*(height/2 - 1) + (width/2)
    

    # parameter data.row_step gives us the amount of bytes in one row
    # data.point_step tells us the amount of bytes for one point. 
    
    # calulcating center of 640x480 RGB image = (320, 240)

    # arraypos = (height/2)*data.row_step + (width/2)*data.point_step
    # arrayPosX = arraypos + data.fields[0].offset # X offset = 0
    # arrayPosY = arraypos + data.fields[1].offset # Y offset = 4
    # arrayPosZ = arraypos + data.fields[2].offset # Z offset = 8

    # print ([arrayPosX, arrayPosY, arrayPosZ])
    print()



def talker():
    rospy.init_node('distance', anonymous=True)
    # rospy.Subscriber("/depthcam/depth/points", PointCloud2, callbackfn)
    rospy.Subscriber("/depthcam/depth/image_raw", Image, callbackfn)
    rospy.spin()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass    