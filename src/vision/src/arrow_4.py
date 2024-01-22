#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String


cap = cv2.VideoCapture(0)


def talker():
    rospy.init_node('arrow', anonymous=True)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break

        
        img = frame
        #convert the image to grayscale
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        #apply canny edge detection to the image
        edges = cv2.Canny(gray,50,150,apertureSize = 3)

        # cv2.imshow("canny'd image", edges)        

        # ------------------------------------------------------------------------------------------
        
        # Set a suitable threshold “T”.Otherwise,it will loss some arrow information or keep the unwanted objects in the environment.
        
        # DETECTING WHITE

        # Get three primary color component value of each pixel’s RGB,which are decoded from the output
        # When (R-G)>T and (R-B)>T,the value of R,G,B are all set to 255.
        # In other cases, R,G and B are all set to zero.
        # As the values of R,G,B are same,color image is grayscale. When the values of R,G,B are 255 > white
    

        # DETECTING DIRECTION

        # find the minimum and maximum point in the X and Y directions

        # ------------------------------------------------------------------------------------------


        img_left = cv2.imread('/home/alexii/robotics/onebot_ws/src/vision/src/left.png', 0)
        img_right = cv2.imread('/home/alexii/robotics/onebot_ws/src/vision/src/right.png', 0)
        # img_down = cv2.imread('down.png', 0)
        # img_up = cv2.imread('up.png', 0)
        
        (h_to, w_to) = img_left.shape
        (h_from, w_from) = edges.shape

        # ------ RESIZING ORIGINAL IMAGE
        # edges_resized = cv2.resize(edges, (w_to, h_to), interpolation= cv2.INTER_LINEAR)

        # ------- CROPPING ORIGINAL IMAGE
        h_new = int((h_from - h_to)/2)
        w_new = int((w_from - w_to)/2)
        edges_cropped = edges[h_new:h_new+h_to, w_new:w_new+w_to]


        cv2.imshow("cropped image", edges_cropped)


        # ------- USING ABSOLUTE DIFFERENCE

        res_left = cv2.absdiff(edges_cropped, img_left)
        res_left = res_left.astype(np.uint8)

        res_right = cv2.absdiff(edges_cropped, img_right)
        res_right = res_right.astype(np.uint8)

        percentage_left = 100 - (np.count_nonzero(res_left) * 100)/ res_left.size
        percentage_right = 100 - (np.count_nonzero(res_right) * 100)/ res_right.size


        # ------- USING CONTOUR SIMILARITY
        # percentage_left = 100 - cv2.matchShapes(img_left, edges_cropped, 1, 0.0)
        # percentage_right = 100 - cv2.matchShapes(img_right, edges_cropped, 1, 0.0)


        if(percentage_left > percentage_right):
            print("RIGHT")
        else:
            print("LEFT")

        # ret = cv2.matchShapes(img_left, img_right, 1, 0.0)
        # print(ret)

        # print("similarity to left: ",percentage_left)
        # print("similarity to right: ",percentage_right)
        # print()


        # press esc to exit
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
