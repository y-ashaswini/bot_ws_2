#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
import time

def fake_gps_publisher():
    rospy.init_node('fake_gps_publisher', anonymous=True)
    gps_publisher = rospy.Publisher('/fake_gps_data', PointStamped, queue_size=10)
    rate = rospy.Rate(1) 

    latitude = 40.009
    longitude = 60.985

    while not rospy.is_shutdown():
        gps_data = PointStamped()
        gps_data.header = Header()
        gps_data.header.stamp = rospy.Time.now()
        gps_data.point.x = longitude
        gps_data.point.y = latitude
        gps_data.point.z = 0.0  

        rospy.loginfo("Publishing Fake GPS Data - Latitude: {}, Longitude: {}".format(latitude, longitude))
        gps_publisher.publish(gps_data)

        latitude += 0.01
        longitude += 0.01

        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.loginfo("Starting fake gps publisher...")
        fake_gps_publisher()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
