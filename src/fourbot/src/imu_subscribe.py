#!/usr/bin/env python
import rospy 
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

def callback (msg):
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    [roll, pitch, yaw] = euler_from_quaternion (orientation_list)
    print([roll, pitch, yaw])

def listener():
	rospy.init_node("imu_nav",anonymous=True)
	rospy.Subscriber("/imu", Imu, callback)
	rospy.spin()

if __name__=="__main__":
	listener()	
        