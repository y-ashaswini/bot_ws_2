#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix  # Import NavSatFix message

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' GPS Data: %s', data.data)

def arduino_com():
    arduino = serial.Serial('/dev/ttyUSB0', 9600)
    rospy.init_node('gps', anonymous=True)
    pub = rospy.Publisher('gps_data', NavSatFix, queue_size=10)  # Change message type to NavSatFix
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        if arduino.in_waiting > 0:
            gps_data = arduino.readline().decode().strip().split(' ')
            
            if len(gps_data) == 2:  # Assuming latitude and longitude
                latitude, longitude = map(float, gps_data)
                
                # Create NavSatFix message
                navsatfix_msg = NavSatFix()
                navsatfix_msg.latitude = latitude
                navsatfix_msg.longitude = longitude

                rospy.loginfo("Publishing GPS data: {}".format(navsatfix_msg))
                pub.publish(navsatfix_msg)

            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.loginfo("Starting gps node publisher...")
        arduino_com()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
