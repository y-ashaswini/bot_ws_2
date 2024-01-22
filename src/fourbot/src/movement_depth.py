#!/usr/bin/env python

# Will not work unless depthcam is un-commented in fourbot.gazebo and fourbot.xacro

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep

class Movement():
    def __init__(self):
        rospy.init_node('rover_move', anonymous=True)
        self.vel_pub = rospy.Publisher('/fourbot/cmd_vel', Twist, queue_size=1)

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        self.too_close = False
        self.collided = False
        self.arrow_detected = False
        self.arrow_direction = "x"

        
        # self.forward_distance_sub = rospy.Subscriber("/depthcam/color/distance_opencv",String ,self.distanceCallback)
        self.forward_distance_sub = rospy.Subscriber("/depthcam/color/distance_opencv", String ,self.testDistCallback)
        self.arrow_detection_sub = rospy.Subscriber("/arrow", String, self.arrowCallback)
        self.command = Twist()
        self.rate = rospy.Rate(10)
        self.end = False
        rospy.on_shutdown(self.shutdownhook)

    def odomCallback (self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)
        print (self.yaw)

    def arrowCallback(self, data):
        if self.arrow_detected == False:
            if(data.data == 'x'):
                self.arrow_detected = False
            else:
                print("detected arrow!")
                self.arrow_detected = True
                self.arrow_direction = data.data    


    def collisionRecovery(self):
        if(self.collided == True):
            t0 = rospy.Time.now().secs
            t1 = t0

            print("current vel: "+str(self.command.linear.x)+", "+str(self.command.linear.y))
            self.command.linear.y = self.command.linear.y*-1
            self.command.linear.x = self.command.linear.x*-1
            original_angular_vel = self.command.angular.z
            self.command.angular.z = -0.5

            print("after collision vel: "+str(self.command.linear.x)+", "+str(self.command.linear.y))

            while(t1 - t0 < 2):
                print("in recovery loop for seconds:",t1-t0)
                t1 = rospy.Time.now().secs
                self.vel_pub.publish(self.command)  
                self.rate.sleep()
            
            self.command.linear.y = self.command.linear.y*-1
            self.command.linear.x = self.command.linear.x*-1
            self.command.angular.z = original_angular_vel
            print("back to vel: "+str(self.command.linear.x)+", "+str(self.command.linear.y))
            self.collided = False


    def checkCollision(self, data):
        if(data == 'nan'):
            print("collided bro")
            self.collided = True


    def arrowTurn(self):
        # check direction 
        if(self.arrow_direction == 'left'):
            self.rotate(angle_deg=90, clockwise=False)
        elif(self.arrow_direction == 'right'):
            self.rotate(angle_deg=90, clockwise=True)
        sleep(3)
        self.arrow_detected = False
        self.arrow_direction = 'x'

    def testDistCallback(self, data):
      if(self.arrow_detected):
            # arrow detected, need to stand close to it for 10 seconds
            print("calling sleep from test dist callback")
            self.sleep_rover(5)                    
            self.arrowTurn()
            

    def distanceCallback(self, data):
        if self.collided == False:
            self.checkCollision(data.data)
            if(data.data < '1.2'):
                if(self.arrow_detected):
                    # arrow detected, need to stand close to it for 10 seconds
                    self.sleep_rover(5)                    
                    self.arrowTurn()

                else:
                    self.too_close = True
            else:
                self.too_close = False
        else:
            pass


    def publish_cmdvel(self):
        while not self.end:
            connections = self.vel_pub.get_num_connections()
            # get the number of connections to other ROS nodes for this topic. For a Publisher, this corresponds to the number of nodes subscribing. For a Subscriber, the number of publishers.

            if connections > 0:
                self.vel_pub.publish(self.command)
                break
            else:
                self.rate.sleep()


    def shutdownhook(self):
        self.stop_rover()
        self.end = True

    def stop_rover(self):
        rospy.loginfo("Shutting down rover!")
        self.command.linear.x = 0.0 # no linear movement
        self.command.linear.y = 0.0 
        self.command.linear.z = 0.0 
        # self.command.angular.x = 0.0
        # self.command.angular.y = 0.0
        self.command.angular.z = 0 # no angular movement
        
        self.rate.sleep()
        
        self.publish_cmdvel() # trigger shutdown
        

    def convert_degree_rad(self, angular_speed_deg, angle_deg):
        angular_speed_rad = angular_speed_deg * 3.14 / 180
        angle_rad = angle_deg * 3.14 / 180
        return [angular_speed_rad, angle_rad]

    def forward_step(self, forward_time = 5):
        self.command.linear.x = 5
        self.command.linear.y = 2

        t0 = rospy.Time.now().secs
        t1 = t0
        while(t1 - t0 != forward_time):
            self.vel_pub.publish(self.command)            
            t1 = rospy.Time.now().secs
            self.rate.sleep()

    def backward_step(self, backward_time = 5):
        self.command.linear.x = -5
        self.command.linear.y = -2

        t0 = rospy.Time.now().secs
        t1 = t0

        while(t1 - t0 < backward_time):
            self.vel_pub.publish(self.command)            
            t1 = rospy.Time.now().secs
            self.rate.sleep()    


    def sleep_rover(self, time_period):
        linear_x = self.command.linear.x
        linear_y = self.command.linear.y
        linear_z = self.command.linear.z
        angular_z = self.command.angular.z
        self.command.linear.x = 0
        self.command.linear.y = 0
        self.command.linear.z = 0
        self.command.angular.x = 0
        self.command.angular.y = 0
        self.command.angular.z = 0

        t0 = rospy.Time.now().secs
        t1 = t0

        while(t1 - t0 < time_period):
            print("sleeping for seconds:",t1-t0)
            t1 = rospy.Time.now().secs
            self.vel_pub.publish(self.command)  
            self.rate.sleep() 

        self.command.linear.x = linear_x
        self.command.linear.y = linear_y
        self.command.linear.z = linear_z
        self.command.angular.z = angular_z


    def forward_continuous(self):
        print("moving forward")
        while not self.too_close and not self.collided:
        # while not self.collided:
            self.vel_pub.publish(self.command)
            self.rate.sleep()    


    def rotate(self, angular_speed_deg =30, angle_deg = 30, clockwise = False):
        self.angular_speed_deg = angular_speed_deg
        self.angle_deg = angle_deg
        self.clockwise = clockwise
        linear_x = self.command.linear.x
        linear_y = self.command.linear.y
        linear_z = self.command.linear.z
        self.command.linear.x = 0
        self.command.linear.y = 0
        self.command.linear.z = 0
        self.command.angular.x = 0
        self.command.angular.y = 0
        [angular_speed_rad, angle_rad] = self.convert_degree_rad(self.angular_speed_deg, self.angle_deg)

        if(self.clockwise):
            self.command.angular.z = -abs(angular_speed_rad)
        else:
            self.command.angular.z = abs(angular_speed_rad)

        t0 = rospy.Time.now().secs

        curr_angle = 0
        
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while(curr_angle < angle_rad):
            self.vel_pub.publish(self.command)
            t1 = rospy.Time.now().secs
            curr_angle = angular_speed_rad*(t1 - t0)

            self.rate.sleep()
        sleep(2)
        
        self.command.angular.z = 0
        self.command.linear.x = linear_x
        self.command.linear.y = linear_y
        self.command.linear.z = linear_z
        
        # after taking a turn, set velocity to zero to stop the robot
        # self.stop_rover()


def wrapper():
    controller = Movement()
    controller.command.linear.x = 10
    controller.command.linear.y = 2

    while not controller.end:
        while not controller.too_close and not controller.collided:
            controller.forward_continuous()

        if controller.collided:
            controller.collisionRecovery()
            controller.forward_continuous()

        elif controller.too_close:
            print("too close, from forward continuous")
            controller.rotate(angle_deg = 60)
            sleep(3)
            controller.forward_continuous()
        else:
            print("anomally end")
            controller.end = True
    
    controller.stop_rover()


if __name__ == '__main__':
    try:
        wrapper()        
    except rospy.ROSInterruptException:
        pass
