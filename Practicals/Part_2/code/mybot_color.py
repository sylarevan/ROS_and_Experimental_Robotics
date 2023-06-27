#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

# Callback function for reading turtlesim node output
def read_pose_callback(msg):
    
    # First step: display read position on the terminal
    # TO BE COMPLETED


if __name__ == '__main__':
    try:
        
        # CODE TO BE COMPLETED

        # And then ... wait for the node to be terminated
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
