#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math

def main():
    rospy.init_node('cmd_another', anonymous=True)
    rospy.Subscriber("/bob/cmd_vel", Twist, callback)
    rospy.spin()

def callback(data):
    # print(data)
    # print("hi")
    
    cmd_vel_pub = rospy.Publisher('/bob/another_cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)

    # while not rospy.is_shutdown():
    # cmd_vel_msg = Twist()
    # cmd_vel_msg.linear.x = math.sqrt(data.linear.x*data.linear.x + data.linear.y*data.linear.y)
    # cmd_vel_msg.angular.z = data.angular.z


    linear_x = data.linear.x
    linear_y = data.linear.y

    # Calculate new linear x and angular y velocities
    new_linear_x = math.sqrt(linear_x**2 + linear_y**2) * (1.0 if linear_x >= 0 else -1.0)
    
    # Calculate the additional angular component based on linear y and linear x
    additional_angular_z = math.atan2(linear_y, linear_x)

    # Sum the original angular z with the additional angular component
    new_angular_z = data.angular.z + additional_angular_z

    # Create a new Twist message with the calculated values
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = new_linear_x
    cmd_vel_msg.angular.z = new_angular_z

    # rospy.loginfo(cmd_vel_msg)

    cmd_vel_pub.publish(cmd_vel_msg)
    rate.sleep()

    # print(cmd_vel_msg)

    cmd_vel_pub.publish(cmd_vel_msg)
    rate.sleep()

if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass
