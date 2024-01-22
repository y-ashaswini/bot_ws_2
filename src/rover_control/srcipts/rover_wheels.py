#!/usr/bin/python
from serial import Serial
import rospy
from rover_control.msg import rover_wheels

ser = Serial("/dev/ttyACM0", 9600)


def callback(data):
    pass

    # if data.action == 1:
    #     ser.write("s" + str(data.speed) + "\n")
    #     print("New Straight PWM = " + str(data.speed))
    # elif data.action == 2:
    #     ser.write("t" + str(data.speed) + "\n")
    #     print("New Turn PWM = " + str(data.speed))
    # else:
    #     ser.write(str(data.action) + "\n")
    #     print(data.action)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("rover/rover_wheels", rover_wheels, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
