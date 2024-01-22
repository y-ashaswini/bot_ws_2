#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
# from cv_bridge import CvBridge, CvBridgeError


class camera1:
    def __init__(self):
        self.img_subscriber = rospy.Subscriber("/fourbot/os1_cloud_node/points", PointCloud2, self.img_sub_callback)
        self.direction_publisher = rospy.Publisher("/arrow", String, queue_size=1)
        self.arrow_q =['x','left','x','right','x']
        self.arrow_index = 0
        self.arrow_dir = self.arrow_q[self.arrow_index]
        self.init_time = rospy.Time.now().secs
        self.time_interval = 10

    def img_sub_callback(self, data):
        curr_time = rospy.Time.now().secs
        if(curr_time - self.init_time == self.time_interval):
            self.init_time = curr_time
            self.arrow_index = (self.arrow_index+1)%5
            self.arrow_dir = self.arrow_q[self.arrow_index]


        # bridge = CvBridge()
        
        # try:
        #     cv_img = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        # except CvBridgeError as e:
        #     rospy.logerr(e)

        # img = cv_img

        # cv2.imshow('ArrowCamera', img)

        # img = cv2.resize(img, (224, 224))
        # img = np.asarray(img)
        # img = np.expand_dims(img, axis=0)
        
        print("direction:",self.arrow_dir)
        self.direction_publisher.publish(self.arrow_dir)

        cv2.waitKey(3)

def main():
    camera1()
        
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    
    # cv2.destroyAllWindows()
    


if __name__ == "__main__":
    try:
        rospy.init_node('arrow', anonymous=True)
        main()
    except rospy.ROSInterruptException:
        pass
