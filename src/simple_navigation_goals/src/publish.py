#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time


def main():
    rospy.init_node('twist_mid', anonymous=True)
    # Create a publisher for the "cmd_vel" topic
    cmd_vel_pub = rospy.Publisher('/bob/cmd_vel', Twist, queue_size=10)

    # # Set the publishing rate (e.g., 1 Hz)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        cmd_vel_msg = Twist()
        # cmd_vel_msg.linear.x = -0.106
        # cmd_vel_msg.angular.z = 0.177
        cmd_vel_msg.linear.x = -0.2
        cmd_vel_msg.angular.z = -0.1

        print(cmd_vel_msg)

        cmd_vel_pub.publish(cmd_vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
        # rospy.Subscriber("/bob/cmd_vel", Twist, callback)
    except rospy.ROSInterruptException:
        pass



# 2 sec done, sending 0 0,
# 2 sec done, sending -229 -134,
# 2 sec done, sending -8 -164,
# 2 sec done, sending 19 -156,
# 2 sec done, sending -65 -168,
# 2 sec done, sending -3 -169,
# 2 sec done, sending -226 -136,
# 2 sec done, sending -141 188,
# 2 sec done, sending -106 177,
# 2 sec done, sending 302 -171,
# 2 sec done, sending 316 -181,
# 2 sec done, sending -269 157,
# 2 sec done, sending -231 123,
# 2 sec done, sending -63 107,
# 2 sec done, sending -163 114,
# 2 sec done, sending 112 135,
# 2 sec done, sending -82 101,
# 2 sec done, sending -196 94,
# 2 sec done, sending -58 200,
# 2 sec done, sending -183 167,
# 2 sec done, sending -272 176,
# 2 sec done, sending 118 168,
# 2 sec done, sending -3 192,
# 2 sec done, sending -245 159,
# 2 sec done, sending 383 -209,
# 2 sec done, sending -322 218,
# 2 sec done, sending -249 163,
# 2 sec done, sending 284 162,
# 2 sec done, sending 302 -55,
# 2 sec done, sending 324 -166,
# 2 sec done, sending 247 126,
# 2 sec done, sending 299 -189,
# 2 sec done, sending -208 107,
# 2 sec done, sending 315 -73,
# 2 sec done, sending -245 20,
# 2 sec done, sending 319 -168,
# 2 sec done, sending 222 112,
# 2 sec done, sending -56 153,
# 2 sec done, sending -200 98,
# 2 sec done, sending -215 102,
# 2 sec done, sending -233 122,
# 2 sec done, sending -259 130,
# 2 sec done, sending -44 190,
# 2 sec done, sending 339 192,
# 2 sec done, sending 326 160,
# 2 sec done, sending 324 -41,
# 2 sec done, sending 358 -160,
# 2 sec done, sending 299 173,
# 2 sec done, sending 322 166,
# 2 sec done, sending 319 -170,
# 2 sec done, sending 324 167,
# 2 sec done, sending 322 129,
# 2 sec done, sending 323 -37,
# 2 sec done, sending 324 166,
# 2 sec done, sending 325 -167,
# 2 sec done, sending -180 227,
# 2 sec done, sending 320 165,
# 2 sec done, sending 324 61,
# 2 sec done, sending 322 140,
# 2 sec done, sending 294 -207,
# 2 sec done, sending 299 200,
# 2 sec done, sending 324 118,
# 2 sec done, sending 322 -165,
# 2 sec done, sending 321 -155,
# 2 sec done, sending 249 190,
# 2 sec done, sending 292 185,
# 2 sec done, sending 243 109,
# 2 sec done, sending 352 -204,
# 2 sec done, sending 344 60,
# 2 sec done, sending -182 109,
# 2 sec done, sending -219 115,
# 2 sec done, sending 247 148,
# 2 sec done, sending 275 165,
# 2 sec done, sending 211 227,
# 2 sec done, sending 301 178,
# 2 sec done, sending 295 192,
# 2 sec done, sending 321 -165,
# 2 sec done, sending 327 132,
# 2 sec done, sending 323 14,
# 2 sec done, sending 321 161,
# 2 sec done, sending 315 -162,
# 2 sec done, sending 265 156,
# 2 sec done, sending 263 157,
# 2 sec done, sending 350 -37,
# 2 sec done, sending 343 -2,
# 2 sec done, sending 393 224,
# 2 sec done, sending 324 -167,
# 2 sec done, sending 298 127,
# 2 sec done, sending 362 -197,
# 2 sec done, sending 334 255,
# 2 sec done, sending 279 -143,
# 2 sec done, sending 240 143,
# 2 sec done, sending 245 20,
# 2 sec done, sending 373 -214,
# 2 sec done, sending -217 94,
# 2 sec done, sending -289 47,
# 2 sec done, sending -213 61,
# 2 sec done, sending -64 121,
# 2 sec done, sending 234 126,
# 2 sec done, sending 209 95,
# 2 sec done, sending -219 37,
# 2 sec done, sending -151 85,
# 2 sec done, sending 223 93,
# 2 sec done, sending -246 131,
# 2 sec done, sending 235 142,
# 2 sec done, sending -70 160,
# 2 sec done, sending -141 169,
# 2 sec done, sending -218 175,
# 2 sec done, sending 189 181,
# 2 sec done, sending 261 70,
# 2 sec done, sending 341 -18,
# 2 sec done, sending 328 -91,
# 2 sec done, sending -242 152,
# 2 sec done, sending -260 142,
# 2 sec done, sending 253 -34,
# 2 sec done, sending 229 137,
# 2 sec done, sending -306 191,
# 2 sec done, sending 350 -115,
# 2 sec done, sending -95 150,
# 2 sec done, sending 294 -24,
# 2 sec done, sending -9 234,
# 2 sec done, sending -268 168,
# 2 sec done, sending 61 -126,
# 2 sec done, sending -539 -275,
# 2 sec done, sending -459 -74,
# 2 sec done, sending -354 -85,
# 2 sec done, sending -236 130,
# 2 sec done, sending -208 102,
# 2 sec done, sending -217 110,
# 2 sec done, sending -223 114,
# 2 sec done, sending -253 133,
# 2 sec done, sending -253 140,
# 2 sec done, sending -226 112,
# 2 sec done, sending -226 141,
# 2 sec done, sending -208 104,
# 2 sec done, sending -233 116,
# 2 sec done, sending -221 110,
# 2 sec done, sending -290 161,
# 2 sec done, sending -201 96,
