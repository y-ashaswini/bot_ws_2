#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
from tensorflow import keras

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

model = keras.models.load_model("/home/alexii/robotics/onebot_ws/src/vision/Model")

def array2dir(array):
    if array[0][0] > array[0][1] and array[0][0] > array[0][2]:
        return "right"
    elif array[0][1] > array[0][0] and array[0][1] > array[0][2]:
        return "left"
    elif array[0][2] > array[0][1] and array[0][2] > array[0][0]:
        return "up"
    else:
        return "x"

class camera1:
    def __init__(self):
        self.img_subscriber = rospy.Subscriber("/depthcam/color/image_raw", Image, self.img_sub_callback)
        self.direction_publisher = rospy.Publisher("/arrow", String, queue_size=1)
        self.external_q = []

    def decideDirection(self, q):
        # q = [r, r, r, r, l, l, u, u]
        # direction = r
        d = {'l': 0, 'r': 0}
        for i in q:
            if i != 'u':
                d[i]+=1
        
        max_num = 0
        max_dir = 'x'
        for dir, num in d.items():
            if(num >= max_num):
                max_no = num
                max_dir = dir
        
        print("direction is "+max_dir)
        return max_dir


    def img_sub_callback(self, data):
        bridge = CvBridge()
        
        try:
            cv_img = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        img = cv_img

        # cv2.imshow('ArrowCamera', img)

        img = cv2.resize(img, (224, 224))

        img = np.asarray(img)
        plt.imshow(img)
        img = np.expand_dims(img, axis=0)
        output = model.predict(img)
        curr_dir = array2dir(output)
        print("direction:",curr_dir)
        self.direction_publisher.publish(curr_dir)
        
        # self.external_q.append(curr_dir)

        # if(len(self.external_q) == 50):
        #     direction = self.decideDirection(self.external_q)
        #     self.direction_publisher.publish(direction)
        #     self.external_q = []

        cv2.waitKey(3)


def main():
    camera1()
        
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    
    cv2.destroyAllWindows()
    


if __name__ == "__main__":
    try:
        rospy.init_node('arrow', anonymous=True)
        main()
    except rospy.ROSInterruptException:
        pass
