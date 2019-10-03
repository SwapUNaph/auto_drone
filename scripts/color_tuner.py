#!/usr/bin/python
'''
Program description: This node sends hsv image to GCS for color tuning.
Author: Swapneel Naphade (snaphade@umd.edu)
version: 2.0
Date: 6/19/19
Change log: 
	7/5/19: Added the gate pose publisher
'''

import rospy
import signal
import sys
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

def signal_handler(_, __):
    # enable Ctrl+C of the script
    sys.exit(0)


if __name__ == '__main__':
	bridge = CvBridge()
	signal.signal(signal.SIGINT, signal_handler)
	
	# Initialize node
	rospy.init_node('color_tuner', anonymous=False)
	
	# Subcribers
	# state subscription
	# rospy.Subscriber('/state', String, state_callback)
	
	# VideoCapture
	cap = cv2.VideoCapture(0)

	# Publishers
	# Image publication
	image_publisher = rospy.Publisher('/camera_tuner_image', Image, queue_size=1)

	
	# Update rate for the control loop
	rate = rospy.Rate(100) # 100 hz
	
	while not rospy.is_shutdown():
		while(cap.isOpened()):
			ret, img = cap.read()		
			if ret:
				height, width, depth = img.shape
				half_width = int(width/2)			
				
				# Separate left and right images
				left_img = img[:, :half_width]
				right_img = img[:, half_width:width]
				
				image_publisher.publish(bridge.cv2_to_imgmsg(left_img, "bgr8"))
				rate.sleep()
	
	
