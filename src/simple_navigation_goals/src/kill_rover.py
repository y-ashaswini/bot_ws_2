#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist


pub = rospy.Publisher('/bob/cmd_vel', Twist, queue_size=10)


def stop_rover():
    twist = Twist()
    s = rospy.Time.now()
    start_time = s.secs
    curr_time = start_time
    rate = rospy.Rate(10)
    while(curr_time - start_time <= 2):
        pub.publish(twist)
        rate.sleep()
        c = rospy.Time.now()
        curr_time = c.secs
    print("rover movement is killed.")


def main():
    rospy.init_node("rover_kill_node")
    stop_rover()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
