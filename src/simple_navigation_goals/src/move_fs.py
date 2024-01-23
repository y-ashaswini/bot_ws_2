#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
from scipy import stats
from std_srvs.srv import Empty

directions = []
depths = []
p = 2  # Default value for 'nan' or no detection
pub = rospy.Publisher('/bob/cmd_vel', Twist, queue_size=10)
countdown_pub = rospy.Publisher('/countdown', Twist, queue_size=10)
counter = 10
countdown_active = False
direction_subscriber = None  # Global variable to store the direction subscriber

start_time = 0
curr_time = start_time
time_interval = 5 # seconds


def callback(data):

    global p, counter, countdown_active

    if countdown_active:
        # Ignore arrow detection during countdown
        return
    
    global start_time, curr_time
    c = rospy.Time.now()
    curr_time = c.nsecs

    l = data.data.split(',')
    direction = l[:len(l)-1]
    depth = l[-1]

    directions.extend(direction)
    depths.append(int(depth))
    

    if(curr_time - start_time >= time_interval):
        # execute turns ONLY if interval is over AND depth is < 1500
    
        mode_result = stats.mode(directions)
        mode_value = mode_result.mode[0]
        print("mode detection:", mode_value)

        d = 0
        depths = [i for i in depths if i>0]
        for i in depths:
            d+=i
        
        d = d/len(depths)

        # d = average depth value

        if mode_value == 'Left':
            p = 0
            start_countdown()
        elif mode_value == 'Right':
            p = 1
        elif mode_value == 'barrel - v1 cone barrel relabel': # needs to be edited. current algorithm shows barrel very often
            p = 3
        else:
            p = 2 # nan, do nothing

        depths = []
        directions = []

        # unsubscribe temporarily from movebase?? overrride movebase path ???
        # or just stop listening to cmd_vel commands from there???
        
        if(d <= 1500 and d > 0 ):
            # ONLY DO THIS IF THE ROVER IS WITHIN THE RADIUS !!
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


def start_move_base():
    rospy.wait_for_service('/move_base/start')
    try:
        start_move_base_proxy = rospy.ServiceProxy('/move_base/start', Empty)
        start_move_base_proxy()
        rospy.loginfo("move_base node started")
    except rospy.ServiceException as e:
        rospy.logerr("Service call to start move_base failed: %s", str(e))

def stop_move_base():
    rospy.wait_for_service('/move_base/stop')
    try:
        stop_move_base_proxy = rospy.ServiceProxy('/move_base/stop', Empty)
        stop_move_base_proxy()
        rospy.loginfo("move_base node stopped")
    except rospy.ServiceException as e:
        rospy.logerr("Service call to stop move_base failed: %s", str(e))


def execute_rover_movement():
    global p, countdown_active

    if p == 0 or p == 1:
        # left or right
        rospy.init_node('start_stop_move_base_override_cmd')
        stop_move_base()
        global direction_subscriber

        # Unsubscribe from the direction topic
        direction_subscriber.unregister()
        
        stop_rover()
        rospy.sleep(10) # stop inside 2m radius for 10 seconds

        # Set up a rate to control the publishing frequency
        rate = rospy.Rate(10)
        twist_turn = Twist()


        if p == 0 and countdown_active:
            # left turn
            print("detected left, executing left turn")
            twist_turn.angular.z = 1.0
        
        elif p == 1 and countdown_active:
            # right turn
            print("detected right, executing right turn")
            twist_turn.angular.z = -1.0

        turn_duration = rospy.Duration.from_sec(1.5)  # 90 degrees in radians
        start_time_turn = time.time()

        while time.time() - start_time_turn < turn_duration.to_sec():
            pub.publish(twist_turn)
            rate.sleep()

        stop_rover()
        countdown_active = False  # Stop the countdown after turning

        global p
        p=2 # set it as 'nan' again, until next detection


        # restart movebase 
        start_move_base()


        # Subscribe again to the direction topic
        direction_subscriber = rospy.Subscriber("/direction", String, callback)


    elif p == 3:
        print("detected goal, stopping")
        # do something when goal
        stop_move_base()
        stop_rover()

    else:
        print("detected nothing")



def move_rover():
    twist = Twist()
    twist.linear.x = 0.2
    pub.publish(twist)


def stop_rover():
    twist = Twist()
    pub.publish(twist)


def list_switcher_node():
    global start_time, curr_time
    s = rospy.Time.now()
    start_time = s.secs
    curr_time = start_time

    global direction_subscriber
    rospy.init_node("list_switcher_node")
    direction_subscriber = rospy.Subscriber("/direction", String, callback)
    rospy.spin()


if __name__ == "__main__":
    try:
        list_switcher_node()
    except rospy.ROSInterruptException:
        pass
