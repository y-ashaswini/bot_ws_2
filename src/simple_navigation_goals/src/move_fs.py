#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
import numpy as np
from scipy import stats

lists = [np.array([], dtype=object) for _ in range(5)]
p = 2  # Default value for 'no detection'
pub = rospy.Publisher('/bob/cmd_vel', Twist, queue_size=10)
countdown_pub = rospy.Publisher('/countdown', Twist, queue_size=10)
counter = 10
countdown_active = False
direction_subscriber = None  # Global variable to store the direction subscriber

def callback(data):
    global p, counter, countdown_active

    if countdown_active:
        # Ignore arrow detection during countdown
        return

    lists[0] = np.append(lists[0], data.data)

    flattened_array = np.concatenate(lists)
    lists[1:] = lists[:-1]
    lists[0] = np.array([], dtype=object)

    if len(flattened_array) > 0:
        mode_result = stats.mode(flattened_array)
        mode_value = mode_result.mode[0]
        print("Mode:", mode_value)

        if mode_value == 'Left Arrow':
            p = 0
            start_countdown()
        elif mode_value == 'Right Arrow':
            p = 1
        elif mode_value == 'no detection':
            p = 2
        else:
            p = 2

        execute_rover_movement()

def start_countdown():
    global counter, countdown_active
    counter = 10
    countdown_active = True
    publish_countdown()

def publish_countdown():
    global counter
    twist = Twist()

    for counter in range(10, 0, -1):
        twist.linear.x = counter
        countdown_pub.publish(twist)
        rospy.sleep(1)

    # Stop the countdown by publishing an empty Twist message
    countdown_pub.publish(Twist())

def execute_rover_movement():
    global p, countdown_active

    if p == 0 and countdown_active:
        move_and_turn_left()
    elif p == 1:
        move_and_turn_right()
    elif p == 2:
        move_rover()
    else:
        stop_rover()

def move_and_turn_left():
    global countdown_active, direction_subscriber

    # Unsubscribe from the direction topic
    direction_subscriber.unregister()

    stop_rover()
    rospy.sleep(5)

    # Set up a rate to control the publishing frequency
    rate = rospy.Rate(10)

    twist_turn = Twist()
    twist_turn.angular.z = 1.0
    turn_duration = rospy.Duration.from_sec(1.5)  # 90 degrees in radians
    start_time_turn = time.time()

    while time.time() - start_time_turn < turn_duration.to_sec():
        pub.publish(twist_turn)
        rate.sleep()

    stop_rover()
    countdown_active = False  # Stop the countdown after turning
    p=2

    # Subscribe again to the direction topic
    direction_subscriber = rospy.Subscriber("/direction", String, callback)

def move_and_turn_right():
    global direction_subscriber

    # Unsubscribe from the direction topic
    direction_subscriber.unregister()

    # Turn right
    twist_turn = Twist()
    twist_turn.angular.z = -1.0
    turn_duration = rospy.Duration.from_sec(1.5)  # 90 degrees in radians
    start_time_turn = time.time()

    # Set up a rate to control the publishing frequency
    rate = rospy.Rate(10)

    while time.time() - start_time_turn < turn_duration.to_sec():
        pub.publish(twist_turn)
        rate.sleep()

    stop_rover()
    p=2

    # Subscribe again to the direction topic
    direction_subscriber = rospy.Subscriber("/direction", String, callback)

def move_rover():
    twist = Twist()
    twist.linear.x = 0.2
    pub.publish(twist)

def stop_rover():
    twist = Twist()
    pub.publish(twist)

def switch_lists(event):
    pass

def list_switcher_node():
    global direction_subscriber
    rospy.init_node("list_switcher_node")
    direction_subscriber = rospy.Subscriber("/direction", String, callback)
    timer = rospy.Timer(rospy.Duration(1), switch_lists)  # Switch lists every 5 seconds
    rospy.spin()

if __name__ == "__main__":
    try:
        list_switcher_node()
    except rospy.ROSInterruptException:
        pass
