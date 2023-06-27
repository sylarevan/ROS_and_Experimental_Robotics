#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv_bridge
import cv2

cvBridge = cv_bridge.CvBridge()

# Transform the image to openCV format, msg is the original image from ROS
cvImage = cvBridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

# Change color represtation from BGR to HSV
hsv = cv2.cvtColor(cvImage, cv2.COLOR_BGR2HSV)

# Image binarisation
mask = cv2.inRange(hsv, ...)

# Compute the mask moments
M = cv2.moments(mask)

if M["m00"] > 0:

    # Calculate x,y coordinate of center
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])

    # Display the image with cv2.imshow, or draw a form with cv2.rectangle or cv2.circle