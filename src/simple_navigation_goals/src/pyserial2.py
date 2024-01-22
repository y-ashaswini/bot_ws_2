#!/usr/bin/env python
import rospy
import serial
from geometry_msgs.msg import Twist

serial_port = '/dev/ttyACM0'
baud_rate = 9600

ser = serial.Serial(serial_port, baud_rate)
start_time = 0
curr_time = 0

def normalize_value(x):
    neg = 1
    if x < 0:
        neg = -1
        x*=-1
    
    return int((x + 0.5)*1000)*neg


def callback(data):
    try: 
        global start_time, curr_time
        c = rospy.Time.now()
        curr_time = c.secs

        # normalised sending
        # temp_x = str(normalize_value(data.linear.x))
        # temp_z = str(normalize_value(data.angular.z))

        # simple sending
        temp_x = str(int(data.linear.x*1000))
        temp_z = str(int(data.angular.z*1000))
        
        value_to_send = temp_x + " " + temp_z + ","
        if(curr_time - start_time >= 2):
            print("2 sec done, sending",value_to_send)
            ser.write(value_to_send.encode())
            t = rospy.Time.now()
            start_time = t.secs
            
    except KeyboardInterrupt:
        print("\nExiting program.")
        ser.close()

def listener():
    rospy.init_node('cmd_listener', anonymous=True)
    global start_time, curr_time
    t = rospy.Time.now()
    start_time = t.secs
    curr_time = start_time
    rospy.Subscriber("bob/cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()
