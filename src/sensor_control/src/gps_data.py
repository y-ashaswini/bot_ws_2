#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import NavSatFix, Imu
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String, UInt8
from sensor_control.msg import gps_msg
import time
import serial

goal_lat = 0
cur_lat = 0
goal_lon = 0
cur_lon = 0
yaw = 0
init_yaw = 0
flag = 0

# ser = serial.Serial("/dev/ttyACM0",9600)


def callback_pose(data):
    global goal_lat, goal_lon, cur_lat, cur_lon, flag
    print("FLAG: ", flag)
    
    if flag:
        msg = gps_msg()
        cur_lat = math.radians(data.latitude)
        cur_lon = math.radians(data.longitude)
        # rospy.loginfo(math.radians(data.latitude))
        pub = rospy.Publisher("/command", gps_msg)
        #####DISTANCE######
        R = 6371000
        del_phi = (goal_lat-cur_lat)
        del_lam = (goal_lon-cur_lon)
        a = (math.sin(del_phi/2))**2+math.cos(cur_lat) * \
            math.cos(goal_lat)*(math.sin(del_lam/2))**2
        c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
        d = R*c

        # rospy.loginfo(d)
        ####BEARING#########
        y = math.sin(goal_lon-cur_lon)*math.cos(goal_lat)
        x = math.cos(cur_lat)*math.sin(goal_lat) - math.sin(cur_lat) * \
            math.cos(goal_lat)*math.cos(goal_lon-cur_lon)
        bearing = math.atan2(y, x)
        # print("BEARING",bearing)
        # print("YAW",yaw)

        corrected_yaw = yaw-init_yaw

        time.sleep(0.1)
        angle = bearing-corrected_yaw
        msg.target_bearing = bearing
        msg.current_bearing = corrected_yaw
        msg.distance_from_target = d

        # DISTANCE 2 SPEED
        if abs(angle) > 0.05:
            if(angle < 0):
                msg.command = 6
            else:
                msg.command = 4
            rospy.loginfo(abs(abs(bearing)-abs(corrected_yaw)))
        else:
            if(d > 0.5):
                msg.command = 8
            else:
                msg.command = 0
        print(msg)
        pub.publish(msg)


def callback_imu(data):
    global yaw, init_yaw, flag
    orientation_q = data.orientation
    orientation_list = [orientation_q.x, orientation_q.y,
                        orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    # if(flag == 0):
    #    init_yaw = yaw
    #    print("yaw initialised")

    #flag = 1


def callback_goal(data):
    global goal_lat, goal_lon, cur_lat, cur_lon, flag
    flag = 1
    # while((data.latitude==goal_lat) and (data.longitude==goal_lon)):
    goal_lat = math.radians(data.latitude)
    goal_lon = math.radians(data.longitude)
    # print("GOAL LAT:",goal_lat)
    # goal_lat = math.radians(data.latitude)
    # goal_lon = math.radians(data.longitude)
    # print("YES!")
    '''pub = rospy.Publisher("/command",UInt8)
	#####DISTANCE######
	if(data):
		goal_lat = math.radians(data.latitude)
		goal_lon = math.radians(data.longitude)
	R = 6371000
	del_phi = (goal_lat-cur_lat)
	del_lam = (goal_lon-cur_lon)
	a = (math.sin(del_phi/2))**2+math.cos(cur_lat)*math.cos(goal_lat)*(math.sin(del_lam/2))**2
	c = 2*math.atan2(math.sqrt(a),math.sqrt(1-a))
	d = R*c
	
	# rospy.loginfo(d)
	####BEARING#########
	y = math.sin(goal_lon-cur_lon)*math.cos(goal_lat)
	x = math.cos(cur_lat)*math.sin(goal_lat) - math.sin(cur_lat)*math.cos(goal_lat)*math.cos(goal_lon-cur_lon)
	bearing = math.atan2(y,x)
	# print("BEARING",bearing)
	# print("YAW",yaw)
	
	corrected_yaw = yaw-init_yaw
	print("Bearing:",corrected_yaw)
	time.sleep(0.1)
	angle = bearing-corrected_yaw
	###DISTANCE 2 SPEED#####
	if(abs(angle)>0.05):
		if(angle<0):
			pub.publish(6)
		else:
			pub.publish(4)
		# time.sleep(0.1)
		# rospy.loginfo(abs(abs(bearing)-abs(corrected_yaw)))
	else:
		if(d>0.5):
			pub.publish(8)
		else:
			pub.publish(0)'''


def listener():
    rospy.init_node("gps_subs", anonymous=True)
    sub1 = rospy.Subscriber("/gps_goal", NavSatFix, callback_goal)

    rospy.Subscriber("/android/fix", NavSatFix, callback_pose)

    sub2 = rospy.Subscriber("/android/imu", Imu, callback_imu)

    rospy.spin()


if __name__ == "__main__":
    listener()
