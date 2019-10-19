#!/usr/bin/python
'''
Program description: The controller takes in the position and heading 
					 errors as input from the path planner and publishes 
					 the cmd_vel to the drone.
Author: Swapneel Naphade (snaphade@umd.edu)
version: 1.0
Date: 6/12/19
Change log: 

'''

import rospy
import signal
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from auto_drone.msg import WP_Msg
from common_resources import *
import numpy as np

# Make the flag true if want to use PID controller
PID_CONTROL = True

def signal_handler(_, __):
    # enable Ctrl+C of the script
    sys.exit(0)

# Initialize PID controllers for x,y,z, roll, pitch and yaw
#PID_x = PID(p=0.1, i=0.0, d=0.5, output_limit = (-2,2))
#PID_y = PID(p=0.1, i=0.0, d=0.5, output_limit = (-2,2))
#PID_z = PID(p=0.3, i=0.0, d=0.5, output_limit = (-2,2))
#PID_yaw = PID(p=0.5, i=0.0, d=0.2, output_limit = (-2,2))
#PID_roll = PID(p=0.5, i=0.0, d=0.2, output_limit = (-0.5,0.5))
#PID_pitch = PID(p=0.5, i=0.0, d=0.2, output_limit = (-0.5,0.5))
	
	
# Cypress 7/25/19
PID_x = PID(p=0.1, i=0.01, d=0.5)
PID_y = PID(p=0.1, i=0.01, d=0.5)
PID_z = PID(p=0.3, i=0.0, d=0.5)
PID_yaw = PID(p=0.5, i=0.0, d=0.2)
#PID_roll = PID(p=0.5, i=0.0, d=0.2, output_limit = (-0.5,0.5))
#PID_pitch = PID(p=0.5, i=0.0, d=0.2, output_limit = (-0.5,0.5))

# Simulation PID
#PID_x = PID(p=1, i=0.0, d=5)
#PID_y = PID(p=1, i=0.0, d=5)
#PID_z = PID(p=1, i=0.0, d=5)
#PID_yaw = PID(p=2, i=0.0, d=0.2)
#PID_roll = PID(p=2, i=0.0, d=0.2)
#PID_pitch = PID(p=2, i=0.0, d=0.2)


def WP_error_callback(WP_error):
	global PID_x, PID_y, PID_z, PID_yaw
	
	pos_error, hdg_error, fmt = WP2array(WP_error)
	
	control_input = Twist()
	
	if PID_CONTROL:
		# Calculate control input from pose error	
		control_input.linear.x = np.sum(PID_x.update(pos_error[0]))
		control_input.linear.y = np.sum(PID_y.update(pos_error[1]))
		control_input.linear.z = np.sum(PID_z.update(pos_error[2]))
		control_input.angular.z = np.sum(PID_yaw.update(hdg_error))
		commander.publish(control_input)
		
	'''
	else:
		position_error, orientation_error = pose2array(pose_error)
		euler_errors =  quat2euler(orientation_error)	
		linear_vel_error, angular_vel_error = twist2array(twist_error)
			
		Kp_position = np.array([1, 1, 1])
		Kd_position = np.array([2, 2, 2])
		
		Kp_attitude = np.array([1, 1, 1])
		Kd_attitude = np.array([0.1, 0.1, 0.1])
		
		control_input_linear = np.multiply(Kp_position, position_error) + np.multiply(Kd_position, linear_vel_error)	 
		control_input_angular = np.multiply(Kp_attitude, euler_errors) + np.multiply(Kd_attitude, angular_vel_error)
		
		control_input.linear.x = control_input_linear[0]
		control_input.linear.y = control_input_linear[1]
		control_input.linear.z = control_input_linear[2]
		control_input.angular.x = control_input_angular[0]
		control_input.angular.y = control_input_angular[1]
		control_input.angular.z = control_input_angular[2]
		commander.publish(control_input)
	'''
	
if __name__ == '__main__':
	
	signal.signal(signal.SIGINT, signal_handler)
	
	
	# Set simulation parameter to True. The system will start in simulation mode by default.
	#SIMULATION = rospy.get_param("/simulation")

	
	# Initialize node
	rospy.init_node('controller', anonymous=False)
	
	# Subcribers
	# State subscription
	#rospy.Subscriber('/state', String, state_callback)
	
	# Corrected drone pose subscription
	#rospy.Subscriber('/drone_pose', Pose, drone_pose_callback)
	
	# Trajectory error subscription
	rospy.Subscriber('/auto/WP_error', WP_Msg, WP_error_callback)
	
	# Publishers
	# cmd_vel publication
	commander = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)
	
	# Update rate for the control loop
	rate = rospy.Rate(50) # 50 hz
	
	while not rospy.is_shutdown():		
		rate.sleep()
	
	
