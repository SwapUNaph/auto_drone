#!/usr/bin/python
'''
Program description: The Fake target publishes artificial coordinates so
				     that the simulation can run.		 
				     
Author: ADR Team
version: 1.0
Date: 7/1/19
Change log: 

'''
import rospy
import sys
from geometry_msgs.msg import Twist, Pose, Quaternion
from nav_msgs.msg import Odometry
import transformations as tfs
import numpy as np
import math
import signal

def signal_handler(_, __):
    # enable Ctrl+C of the script
    sys.exit(0)

def pub_pose_target():
	next_pose = Pose()
	next_pose.orientation.w = 1
	next_pose.position.z = 10
	#rospy.loginfo("Pose error: {}".format(pose_error))
	pose_target_publisher.publish(next_pose)

if __name__ == '__main__':
	
	signal.signal(signal.SIGINT, signal_handler)
	# Initialize node
	rospy.init_node('Fake_target', anonymous=False)
	
	# Publishers
	# pose_error publication
	pose_target_publisher = rospy.Publisher('/pose_target', Pose, queue_size=5)
	
	# Update rate for the control loop
	rate = rospy.Rate(50) # 50 hz
	
	
	while not rospy.is_shutdown():
		pub_pose_target()	
		rate.sleep()
