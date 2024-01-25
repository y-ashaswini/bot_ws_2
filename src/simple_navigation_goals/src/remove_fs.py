#!/usr/bin/env python3


import pickle
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion, PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
import time
# from sensor_msgs.msg import NavSatFix
# from scipy import stats
import statistics
import actionlib
import tf
import math


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


start_time = 0
curr_time = start_time
time_interval = 2 # seconds
ultimate_time = False
ultimate_start_time = 0
first = 0
ultimate_curr_time = ultimate_start_time
gps_file_path = '/home/alexii/robotics/bot_workspace/gps_coordinates.txt'


def callback(data):
    # print("in callback")
    global directions, depths, p, counter, countdown_active, ultimate_curr_time, ultimate_start_time, ultimate_time, first

    if countdown_active:
        # Ignore arrow detection during countdown
        return

    global start_time, curr_time

    c = rospy.Time.now()
    curr_time = c.secs

    if(first == 0):
        first = 1
        ultimate_start_time = curr_time
    ultimate_curr_time = c.secs
    
    if(ultimate_curr_time - ultimate_start_time >= 1200.0):
        ultimate_time = True
        # enable goal detection 

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
            execute_rover_movement()
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
            
            if((p == 1 or p == 0 or p == 3) and d <= 1500 and d > 0):
                # ONLY DO THIS IF THE ROVER IS WITHIN THE RADIUS !!
                execute_rover_movement()




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
    

def stop_move_base():
    cancel_goal_publisher = rospy.Publisher('move_base_simple/cancel', MoveBaseActionGoal, queue_size=10)
    cancel_goal_msg = MoveBaseActionGoal()
    cancel_goal_msg.goal_id.stamp = rospy.Time.now()
    cancel_goal_msg.goal_id.id = ""
    cancel_goal_publisher.publish(cancel_goal_msg)



def execute_rover_movement():
    print("executing movement...")
    global p, countdown_active, current_dir, curr_time

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
            if(current_axis == "x" and current_dir == "f"):
                current_axis = "y"
                current_dir = "b"
            elif(current_axis == "x" and current_dir == "b"):
                current_axis = "y"
                current_dir = "f"
            elif(current_axis == "y" and current_dir == "f"):
                current_axis = "x"
                current_dir = "f"
            elif(current_axis == "y" and current_dir == "b"):
                current_axis = "x"
                current_dir = "b"


        
        elif p == 1 and not countdown_active:
            # right turn
            print("detected right, executing right turn")
            if(current_axis == "x" and current_dir == "f"):
                current_axis = "y"
                current_dir = "f"
            elif(current_axis == "x" and current_dir == "b"):
                current_axis = "y"
                current_dir = "b"
            elif(current_axis == "y" and current_dir == "f"):
                current_axis = "x"
                current_dir = "b"
            elif(current_axis == "y" and current_dir == "b"):
                current_axis = "x"
                current_dir = "f"
            twist_turn.angular.z = -1.0

        countdown_active = True # rotation in motion
        turn_duration = rospy.Duration.from_sec(2)  # 90 degrees in radians
        start_time_turn = time.time()

        while time.time() - start_time_turn < turn_duration.to_sec(2):
            pub.publish(twist_turn)
            rate.sleep()

        stop_rover()
        rospy.sleep(2)
        countdown_active = False  # Stop the countdown after turning

        p = 2 # set it as 'nan' again, until next detection


        try:
            direction_subscriber = rospy.Subscriber("/direction", String, callback)
        except Exception as e:
            print("error subscribing")
        # move_rover()

        
    elif p == 3 and ultimate_time:
        ## GOAL DETECTION MUTED FOR NOW
        # do something when goal
        print("detected goal, stopping")
        stop_rover()
    else:
        p = 2
        print("detected nothing")
        move_rover()
        try:
            direction_subscriber = rospy.Subscriber("/direction", String, callback)
        except Exception as e:
            print("error subscribing")


def get_current_orientation():
    try:
        # Wait for the transform between "base_link" and "map"
        listener = tf.TransformListener()
        listener.waitForTransform("robot_footprint", "map", rospy.Time(), rospy.Duration(4.0))

        # Get the current transformation
        (trans, rot) = listener.lookupTransform("robot_footprint", "map", rospy.Time(0))
        current_x = trans[0]
        current_y = trans[1]
        

        # Convert quaternion to Euler angles
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)

        return str(int(current_x*1000)) + " " + str(int(current_y*1000)) + " " + str(int(yaw*1000))

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Failed to get current orientation.")
        return None


def movebase_goal():
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

def d_movebase_goal(x, y, theta):
    # Create a MoveBase client
    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    move_base_client.wait_for_server()

    # Create a MoveBaseGoal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"  # Adjust the frame_id based on your setup
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set the goal position
    goal.target_pose.pose.position = Point(x, y, 0.0)

    # Set the orientation (yaw angle)
    quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
    goal.target_pose.pose.orientation = Quaternion(*quaternion)

    # Send the goal and wait for result
    move_base_client.send_goal(goal)
    move_base_client.wait_for_result(rospy.Duration(10.0))

    # Print the result
    if move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached successfully!")
        rospy.sleep(1)
    else:
        rospy.loginfo("Failed to reach the goal.")


new_x = 1.0
new_y = 0.0
current_axis = "x"
current_dir = "f"

last_arrow= "nan"

def move_rover():
    global new_x, new_y, current_axis, current_dir
    print("move rover")
    # Set the new goal
    if(current_axis == "x"):
        if(current_dir == "f"):
            new_x +=1.0
        else:
            new_x -=1.0
    else:
        if(current_dir == "f"):
            new_y +=1.0
        else:
            new_y -=1.0
    movebase_goal(new_x, new_y, 1.0)
    # countdown_active = False


def stop_rover():
    twist = Twist()
    global countdown_active
    countdown_active = False
    pub.publish(twist)
 

def init_rotate():
    twist = Twist()
    rate = rospy.Rate(10)
    twist.angular.z = 1.0
    turn_duration = rospy.Duration.from_sec(2)  # 2 (2.5) seconds = 90 degrees in radians
    # so 360 degrees: 8 (10) seconds

    start_time_turn = time.time()

    while time.time() - start_time_turn < turn_duration.to_sec():
        pub.publish(twist)
        rate.sleep()

    twist.angular.z = -1.0
    turn_duration = rospy.Duration.from_sec(4)  # 90 degrees in radians
    start_time_turn = time.time()

    while time.time() - start_time_turn < turn_duration.to_sec():
        pub.publish(twist)
        rate.sleep()
    twist.angular.z = 1.0
    turn_duration = rospy.Duration.from_sec(2)  # 90 degrees in radians
    start_time_turn = time.time()

    while time.time() - start_time_turn < turn_duration.to_sec():
        pub.publish(twist)
        rate.sleep()

    stop_rover()
    rospy.sleep(2)

def list_switcher_node():

    global direction_subscriber
    rospy.init_node("list_switcher_node")

    global start_time, curr_time
    s = rospy.Time.now()
    start_time = s.secs
    curr_time = start_time
    init_rotate()
    direction_subscriber = rospy.Subscriber("/direction", String, callback)
    rospy.spin()


if __name__ == "__main__":
    try:
        list_switcher_node()
    except rospy.ROSInterruptException:
        pass    