#!/usr/bin/env python3


import pickle
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
# from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
import time
from move_base_msgs.msg import MoveBaseActionGoal
# from scipy import stats
import statistics
from std_srvs.srv import Empty


directions = []
depths = []
p = 2  # Default value for 'nan' or no detection
pub = rospy.Publisher('/bob/cmd_vel', Twist, queue_size=10)
countdown_pub = rospy.Publisher('/countdown', Twist, queue_size=10)
goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
counter = 10
countdown_active = False
direction_subscriber = None  # Global variable to store the direction subscriber

gps_sub = None # gps node subscriber
gps_ls = []

start_time = 0
curr_time = start_time
time_interval = 2 # seconds

gps_file_path = '/home/alexii/robotics/bot_workspace/gps_coordinates.txt'


def callback(data):
    # print("in callback")
    global directions, depths, p, counter, countdown_active

    if countdown_active:
        # Ignore arrow detection during countdown
        return

    global start_time, curr_time

    c = rospy.Time.now()
    curr_time = c.secs

    # print("directions now: ")
    # print(directions)
    # print("depths now: ")
    # print(depths)
    # print()

    l = data.data.split(',')
    direction = l[:len(l)-1]
    depth = l[-1]
    directions.extend(direction)
    depths.append(int(depth))
    

    # print(start_time)
    # print(curr_time)
    # print()


    if(curr_time - start_time >= time_interval):
        s = rospy.Time.now()
        start_time = s.secs
        curr_time = start_time

        # execute turns ONLY if interval is over AND depth is < 1500
        print("interval over")
        mode_value = statistics.mode(directions)
        # mode_value = mode_result.mode[0]
        print("mode detection:", mode_value)

        d = 0
        depths = [i for i in depths if i>0]

        if(len(depths)==0):
            p=2
            print("nothing detected")
        else:
            for i in depths:
                d+=i
            d = d/len(depths)
            print("depth detected:",d)

            # d = average depth value

            if mode_value == 'Left':
                p = 0
                # start_countdown()
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


def gps_sub_callback(data):
    # print(data)
    s = str(data.point.x)+" "+str(data.point.y)
    print(s)
    global gps_sub

    with open(gps_file_path, 'a') as file:
        file.write(s+"\n")
    gps_sub.unregister()
    print("saved gps in file")

def save_gps_coordinate():
    print("saving gps...")
    global gps_sub
    gps_sub = rospy.Subscriber('/gps_data', PointStamped, gps_sub_callback)
    
    

def start_move_base():
    rospy.wait_for_service('/move_base/start')
    try:
        start_move_base_proxy = rospy.ServiceProxy('/move_base/start', Empty)
        start_move_base_proxy()
        rospy.loginfo("move_base node started")
    except rospy.ServiceException as e:
        rospy.logerr("Service call to start move_base failed: %s", str(e))


def stop_move_base():
    # rospy.wait_for_service('/move_base/stop')

    # try:
    #     stop_move_base_proxy = rospy.ServiceProxy('/move_base/stop', Empty)
    #     stop_move_base_proxy()
    #     rospy.loginfo("move_base node stopped")
    
    # except rospy.ServiceException as e:
    #     rospy.logerr("Service call to stop move_base failed: %s", str(e))
    cancel_goal_publisher = rospy.Publisher('move_base_simple/cancel', MoveBaseActionGoal, queue_size=10)
    cancel_goal_msg = MoveBaseActionGoal()
    cancel_goal_msg.goal_id.stamp = rospy.Time.now()
    cancel_goal_msg.goal_id.id = ""
    cancel_goal_publisher.publish(cancel_goal_msg)



def execute_rover_movement():
    print("executing movement...")
    global p, countdown_active

    if p == 0 or p == 1:
        # left or right
        print("stopping movebase...")
        stop_move_base()
        print("stopped movebase")
        global direction_subscriber

        # Unsubscribe from the direction topic

        direction_subscriber.unregister()
        print("stopping rover")
        stop_rover()
    
        ###################################################
        ### ADD SOMETHING TO SAVE GPS COORDINATES HERE !!! 
        save_gps_coordinate()

        ###################################################

        print("stop inside for 5 seconds...")
        rospy.sleep(5) # stop inside 2m radius for 10 seconds

        # Set up a rate to control the publishing frequency
        rate = rospy.Rate(10)
        twist_turn = Twist()


        if p == 0 and not countdown_active:
            # left turn
            print("detected left, executing left turn")
            twist_turn.angular.z = 1.0
        
        elif p == 1 and not countdown_active:
            # right turn
            print("detected right, executing right turn")
            twist_turn.angular.z = -1.0

        countdown_active = True # rotation in motion
        turn_duration = rospy.Duration.from_sec(1.5)  # 90 degrees in radians
        start_time_turn = time.time()

        while time.time() - start_time_turn < turn_duration.to_sec():
            pub.publish(twist_turn)
            rate.sleep()

        stop_rover()
        rospy.sleep(2)
        countdown_active = False  # Stop the countdown after turning

        p = 2 # set it as 'nan' again, until next detection


        # restart movebase 
        # start_move_base()
        move_rover()


        # Subscribe again to the direction topic
        direction_subscriber = rospy.Subscriber("/direction", String, callback)

    ## GOAL DETECTION MUTED FOR NOW
    # elif p == 3:
    #     # do something when goal
    #     print("detected goal, stopping")
        
    #     stop_move_base()
    #     stop_rover()

    else:
        p = 2
        print("detected nothing")


def move_rover():
    if not countdown_active:
        # detected no arrow ... is this condition required?

        # twist = Twist()
        # twist.linear.x = 0.2
        # pub.publish(twist)

        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = 0.8
        goal_msg.pose.position.y = 0.0
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0

        rate = rospy.Rate(1)  # 1 Hz

        while not rospy.is_shutdown():
            goal_pub.publish(goal_msg)
            rospy.loginfo("moving 80 cm forward...")

            rate.sleep()


def stop_rover():
    twist = Twist()
    pub.publish(twist)

 

def list_switcher_node():

    global direction_subscriber
    rospy.init_node("list_switcher_node")

    global start_time, curr_time
    s = rospy.Time.now()
    start_time = s.secs
    curr_time = start_time
    
    direction_subscriber = rospy.Subscriber("/direction", String, callback)
    rospy.spin()


if __name__ == "__main__":
    try:
        list_switcher_node()
    except rospy.ROSInterruptException:
        pass    
