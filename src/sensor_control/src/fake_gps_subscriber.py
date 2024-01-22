#!/usr/bin/env python
import rospy 
import math
from sensor_msgs.msg import NavSatFix

def callbackfn(data):
	x = data
	lat = math.radians(x.latitude)
	lon = math.radians(x.longitude)
	print(lat,lon)

def listener():
	rospy.init_node("fake_gps_node",anonymous=True)
	rospy.Subscriber("/fake_gps",NavSatFix, callbackfn)
	rospy.spin()

if __name__=="__main__":
	listener()	
