#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys, termios, tty
import click

# Arrow keys codes
keys = {'\x1b[A':'up', '\x1b[B':'down', '\x1b[C':'right', '\x1b[D':'left', 's':'stop', 'q':'quit'}

if __name__ == '__main__':

    try:    
        # Get character from console
        mykey = click.getchar()
        if mykey in keys.keys():
            char=keys[mykey]

        if char == 'up':    # UP key
            # Do something
        if char == 'down':  # DOWN key
            # Do something
        if char == 'left':  # RIGHT key
            # Do something
        if char == 'right': # LEFT
            # Do something
        if char == "quit":  # QUIT
            # Do something

    except rospy.ROSInterruptException:
        pass