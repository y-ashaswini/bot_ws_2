#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist


def goal_publisher():
    rospy.init_node('goal_publisher', anonymous=True)
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    goal_msg = PoseStamped()
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.header.frame_id = "map"
    goal_msg.pose.position.x = 1.0
    goal_msg.pose.position.y = 0.0
    goal_msg.pose.position.z = 0.0
    goal_msg.pose.orientation.w = 1.0

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        goal_pub.publish(goal_msg)
        rospy.loginfo("Goal published: %s", goal_msg)

        rate.sleep()


def goal_subscriber_callback(msg):
    rospy.loginfo("Received goal: %s", msg)

def goal_subscriber():
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_subscriber_callback)

    # Spin to keep the node alive
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.loginfo("Starting goal publisher...")
        goal_publisher()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
