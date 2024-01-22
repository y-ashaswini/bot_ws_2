#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix

def talker():
	rospy.init_node("goal_publisher",anonymous=True)
	pub = rospy.Publisher("/gps_goal",NavSatFix)
	msg = NavSatFix()
	msg.latitude = 20
	msg.longitude = 30
	while not rospy.is_shutdown():
		pub.publish(msg)

if __name__=="__main__":
	talker()
