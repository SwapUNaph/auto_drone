#!/usr/bin/python
'''
Program description: Point cloud exploration
Author: Swapneel Naphade (snaphade@umd.edu)
version: 1.0
Date: 6/22/19
Change log: 

'''
#############################################################################################
#																							#
# 		Decided not to use point cloud because of the high number of points to filter 		#   
#		and slower pointcloud publish rate. It will be computationally expensive to go		#
#		through all the points, instead	we can calculate the XYZ coordinates of 			#
#		interesting points using depth image and Camera parameters.							#
#																							#
#############################################################################################

import rospy
import signal
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
import numpy as np
import cv2
from time import time
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2

bridge = CvBridge()

def signal_handler(_, __):
    # enable Ctrl+C of the script
    sys.exit(0)

def point_cloud_callback(pcl):
	rospy.loginfo("Width, Height: {}, {}.".format(pcl.width, pcl.height))
	points = list(pc2.read_points(pcl, field_names = ("x", "y", "z", "rgb"), skip_nans=True))
	rospy.loginfo("Number of points: %d" % len(points))
	#for p in points:
		#rospy.loginfo(" x : {}  y: {}  z: {} rgb: {}".format(p[0],p[1],p[2],p[3]))
		

if __name__ == '__main__':
	
	signal.signal(signal.SIGINT, signal_handler)
	
	# Initialize node
	rospy.init_node('point_cloud_estimator', anonymous=False)
	
	# Subcribers
	# state subscription
	#rospy.Subscriber('/state', String, state_callback)
	
	# Camera image subscription
	rospy.Subscriber('/ardrone/stereo/depth/pointcloud', PointCloud2, point_cloud_callback)
	
	# Publishers
	# Image publication
	#gate_detection = rospy.Publisher('/gate_detection', Image, queue_size=2)
	
	# Update rate for the control loop
	rate = rospy.Rate(50) # 50 hz
	
	while not rospy.is_shutdown():		
		rate.sleep()
	
	
