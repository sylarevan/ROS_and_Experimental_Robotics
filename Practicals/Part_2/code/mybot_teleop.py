#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys, termios, tty

# Function to get which key is pressed in the terminal
# ==============================================================
def get_ch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(3)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


if __name__ == '__main__':

    try:    
        # How to use the get_ch function
        char = get_ch()

        if char == "\x1b[A":  # UP key
            # Do something
        if char == "\x1b[B":  # DOWN key
            # Do something
        if char == "\x1b[C":  # RIGHT key
            # Do something
        if char == "\x1b[D":  # LEFT
            # Do something
        if char == "qqq":  # QUIT
            # Do something

    except rospy.ROSInterruptException:
        pass