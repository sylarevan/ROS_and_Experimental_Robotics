#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

# Callback function for reading velocity command
def read_cmd_callback(msg):
	print("---------------------------")
	print("Linear speed: x="+str(msg.linear.x)+", y="+str(msg.linear.y)+", z="+str(msg.linear.z))
	print("Angular speed: x="+str(msg.angular.x)+", y="+str(msg.angular.y)+", z="+str(msg.angular.z))
	print("---------------------------")

if __name__ == '__main__':
    try:
        rospy.init_node('mybot_control', anonymous=False, log_level=rospy.INFO)

        # We subscribe to /cmd_vel topic
        sub = rospy.Subscriber('/cmd_vel', Twist, read_cmd_callback)

        print("Reading command for mybot")
        print("---------------------------")
        
        # And then ... wait for the node to be terminated
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
