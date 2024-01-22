#!/usr/bin/env python
import rospy 
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import math

def callback(data):
	global roll, pitch, yaw
	orientation_q = data.orientation
	(roll, pitch, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
	print(f"roll: {math.degrees(roll)}, pitch: {math.degrees(pitch)}, yaw: {math.degrees(yaw)}")

def listener():
	rospy.init_node("imu_nav", anonymous=True)
	rospy.Subscriber("testrover/imu", Imu, callback)
	rospy.spin()

if __name__=="__main__":
	listener()	
