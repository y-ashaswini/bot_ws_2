#!/usr/bin/env python

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
# from std_msgs.msg import Header
# from datetime import datetime
import rospy

def main(args=None):
    rospy.init_node("fake_gps_node",anonymous=True)
    rate = rospy.Rate(5)
    counter = 0
    gps_values = [{'longitude':57.047218, 'latitude':9.920100, 'altitude':1.15},
                       {'longitude':58.047218, 'latitude':8.920100, 'altitude':2.15},
                       {'longitude':59.047218, 'latitude':7.920100, 'altitude':1.15},
                       {'longitude':60.047218, 'latitude':6.920100, 'altitude':4.15},
                       {'longitude':61.047218, 'latitude':5.920100, 'altitude':6.15},
                       {'longitude':60.047218, 'latitude':6.920100, 'altitude':7.15},
                       {'longitude':59.047218, 'latitude':7.920100, 'altitude':5.15},
                       {'longitude':58.047218, 'latitude':8.920100, 'altitude':3.15}]
    
    pub = rospy.Publisher("/fake_gps",NavSatFix, queue_size=10)
    msg = NavSatFix()
    # msg.header = Header()
    # msg.header.stamp = datetime.now().timestamp()
    # msg.header.frame_id = "fake_gps"

    msg.status.status = NavSatStatus.STATUS_FIX
    msg.status.service = NavSatStatus.SERVICE_GPS

    msg.position_covariance[0] = 0
    msg.position_covariance[4] = 0
    msg.position_covariance[8] = 0
    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

	
    while not rospy.is_shutdown():    
        
        # Position in degrees.
        msg.latitude = gps_values[counter]['latitude']
        msg.longitude = gps_values[counter]['longitude']

        # Altitude in metres.
        msg.altitude = gps_values[counter]['altitude']   

        pub.publish(msg)
        rate.sleep()
        counter = (counter+1)%len(gps_values)


if __name__ == '__main__':    
    main()