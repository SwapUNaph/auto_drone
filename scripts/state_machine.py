#!/usr/bin/env python

#Description	:	High level logic for ADR navigation
#Author 		: 	Swapneel Naphade
#Date 			: 	10/18/2019
#Version 		: 	1.0

import rospy
import signal
import sys
import math
import numpy as np
import time
from std_msgs.msg import Int32, String, Float32MultiArray, MultiArrayDimension, Bool, Float32, Empty
from geometry_msgs.msg import Pose, Twist, Quaternion, Point
from nav_msgs.msg import Odometry
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from auto_drone.msg import WP_Msg, Drone_Pose, Detection_Active
from tf import transformations as tfs
from common_resources import *

class State():
	def __init__(self, next_state=None, pos=None, pos_look=None, tolerance=0.2):
		self.next_state = next_state
		self.pos = pos
		self.pos_look = pos_look
		self.tolerance = tolerance
		self.last_conf_look_hdg = 0
		self.last_conf_pos = None
		
	def advance_next(self, drone_position):
		return ( np.linalg.norm(self.pos - drone_position) < self.tolerance )
		
	def __str__(self):
		return ("pos : {}; pos_look : {}".format(self.pos, self.pos_look))


def signal_handler(_, __):
    # enable Ctrl+C of the script
    sys.exit(0)
    
def reset_odometry():
    global init_pose, drone_pose, ODOMETRY_RESET
    init_pose_position, init_pose_orientation = pose2array(drone_pose)
    init_euler = quat2euler(init_pose_orientation)
    init_euler[0] = 0
    init_euler[1] = 0
    init_pose_orientation = euler2quat(init_euler)
    init_pose = array2pose(init_pose_position, init_pose_orientation)
    
    ODOMETRY_RESET = True
    rospy.loginfo("Odometry Reseted ...")
    
def drone_odometry_callback(drone_odom):
    global drone_pose, init_pose, drone_position, drone_orientation, drone_quaternion, drone_linear_vel, drone_angular_vel
    global FIRST_ODOMETRY
    
    if FIRST_ODOMETRY:
		reset_odometry()
		FIRST_ODOMETRY = False
    else:
		drone_pose = drone_odom.pose.pose
		drone_vel = drone_odom.twist.twist
		drone_pose = pose_diff(drone_pose, init_pose)
		
		drone_position, drone_quaternion  =  pose2array(drone_pose)
		drone_orientation = quat2euler(drone_quaternion)
		drone_linear_vel, drone_angular_vel = twist2array(drone_vel)
    
    
def autonomy_active_callback(autonomy):
	global AUTONOMY_ACTIVE
	AUTONOMY_ACTIVE = autonomy.data
	
	
def gate_detection_callback(gate_WP):
	global drone_position, drone_orientation
	global gate1, gate1_front, gate1_back, gate2, gate2_front, gate2_back
	
	gate_pos_wrt_drone, gate_hdg_wrt_drone, gate_format = WP2array(gate_WP)
	
	# Project gate in track frame
	gate_pos_wrt_track = drone_position + np.matmul( tfs.rotation_matrix(drone_orientation[2], (0,0,1))[:3,:3], gate_pos_wrt_drone )
	gate_pos_wrt_track = gate_pos_MVA.update(gate_pos_wrt_track)
	gate_hdg_wrt_track = drone_orientation[2] + gate_hdg_wrt_drone
	
	#print("\nr_go: {}".format(gate_pos_wrt_track))
	# Get a very good estimate of gate in track (use stdev)
	gate_pos_std = gate_pos_stdev.get_stdev(gate_pos_wrt_track)
	
	#print("Max stdev: {}".format(np.max(gate_pos_std)))
	
	# Set the gate state positions to confident positions
	if np.max(gate_pos_std) < 0.01:
		gate_pos = gate_pos_wrt_track
		gate_pos_front = gate_pos - 3 * np.array([1,0,0])
		#gate_pos_back = gate_pos + 2 * np.array([np.cos(gate_hdg_wrt_track), np.sin(gate_hdg_wrt_track), 0])
		
		if (np.linalg.norm(gate1.pos - gate_pos) < 2):
			gate1.pos = gate_pos
			gate1_front.pos = gate_pos_front
			#gate1_back.pos = gate_pos_back
			print("Gate positions update: gate1 : {}.".format(gate1))
		#elif (np.linalg.norm(gate2.pos - gate_pos) < 3):
			#gate2.pos = gate_pos
			#gate2_front.pos = gate_pos_front
			#gate2_back.pos = gate_pos_back
			
		


def update_state_errors():
	global current_state, drone_orientation, drone_position, drone_quaternion
	
	# Position error in drone frame
	pos_error_wrt_track = current_state.pos - drone_position
	pos_error_wrt_drone = qv_mult( tfs.quaternion_conjugate(drone_quaternion), pos_error_wrt_track )
	
	# Update last confident hdg setpoint
	look_vector = current_state.pos_look - drone_position
	if np.linalg.norm(look_vector) > 5*current_state.tolerance:
		current_state.last_conf_look_hdg = np.arctan2(look_vector[1], look_vector[0])
		
	drone_hdg = np.arcsin(np.sin(drone_orientation[2]))
	
	hdg_error =  np.arcsin(np.sin(current_state.last_conf_look_hdg - drone_hdg))
	
	# Publish the WP error
	WP_error_pub.publish(array2WP(pos_error_wrt_drone, hdg_error, "error"))
	
	
if __name__ == "__main__":
	
	####################### Global Variables ##########################

	# Drone variables
	drone_pose = Pose()
	init_pose = Pose()
	drone_position = np.zeros(3, float)
	drone_orientation = np.zeros(3, float)
	drone_quaternion = np.zeros(4, float)
	drone_linear_vel = np.zeros(3, float)
	drone_angular_vel = np.zeros(3, float)
	AUTONOMY_ACTIVE = False
	FIRST_ODOMETRY = True

	#----Test Mission ----#
	
	look_pos = np.array([15,0,1])
	start = State( pos=np.array([0,0,1]), tolerance=0.05)
	forward = State( pos=np.array([2,0,1]), tolerance=0.05)
	gate1 = State( pos=np.array([5,0,1.7]), tolerance=0.1)
	gate1_front = State( pos=np.array([2,0,1.7]), tolerance=0.1)

	start.pos_look = gate1.pos
	forward.pos_look = gate1.pos
	
	gate1.pos_look = look_pos
	gate1_front.pos_look = gate1.pos


	start.next_state = forward
	forward.next_state = start

	#----------------------#

	'''
	# Main mission (TBD)
	# Define all states
	start = State( )
	wingA = State( )
	gate1_front = State()
	gate1 = State()
	gate1_back = State()
	uturnB1 = State( )
	uturnB2 = State( pos=array2WP([-5, 0.7, 2], 0, "") )
	wingB = State( pos=array2WP([-5, 0.7, 2], 0, "") )
	gate2_front = State()
	gate2 = State()
	gate2_back = State()
	uturnA1 = State( pos=array2WP([-5, 0.7, 2], 0, "") )
	uturnA2 = State( pos=array2WP([-5, 0.7, 2], 0, "") )

	# Connect all states in order
	start.next_state = wingA
	wingA.next_state = gate1_front
	gate1_front.next_state = gate1
	gate1.next_state = gate1_back
	gate1_back.next_state = uturnB1
	uturnB1.next_state = uturnB2
	uturnB2.next_state = wingB
	wingB.next_state = gate2_front
	gate2_front.next_state = gate2
	gate2.next_state = gate2_back
	gate2_back.next_state = uturnA1
	uturnA1.next_state = uturnA2
	uturnA2.next_state = wingA

	# Define look waypoints for all
	start.pos_look = gate1.pos
	wingA.pos_look = gate1.pos
	gate1_front.pos_look = gate1.pos
	gate1.pos_look = gate1_back.pos
	gate1_back.pos_look = uturnB1.pos
	uturnB1.pos_look = uturnB2.pos
	uturnB2.pos_look = gate2.pos
	wingB.pos_look = gate2.pos
	gate2_front.pos_look = gate2.pos
	gate2.pos_look = gate2_back.pos
	gate2_front.pos_look = gate2.pos
	gate2.pos_look = gate2_back.pos
	gate2_back.pos_look = uturnA1.pos
	uturnA1.pos_look = uturnA2.pos
	uturnA2.pos_look = gate1.pos
	'''

	# Set current state to start
	current_state = start

	# STDEV
	gate_pos_stdev = STDEV(50)

	# MVA 
	gate_pos_MVA = MVA(20)

	##################################################################
	
	rospy.init_node("state_machine", anonymous=False)
	
	# Subscribers	
	rospy.Subscriber("/auto/filtered_gate_WP",  WP_Msg,  gate_detection_callback)
	rospy.Subscriber("/bebop/odom",  Odometry,  drone_odometry_callback)
	rospy.Subscriber("/auto/autonomy_active",  Bool, autonomy_active_callback)
	
	# Publishers	
	WP_error_pub = rospy.Publisher("/auto/WP_error", WP_Msg, queue_size = 5)
	land_publisher = rospy.Publisher('/bebop/land', Empty, queue_size=1)
	
	# Wait for initial odometry readings
	time.sleep(5.0)
	
	Loop_rate = rospy.Rate(50) # 50 hz
	
	while not rospy.is_shutdown():
		
		if AUTONOMY_ACTIVE:
			print( "Current Position: {} \n Going to : {}\n\n".format(drone_position, current_state))
			# Update state machine	
			if current_state.advance_next(drone_position):
				current_state = current_state.next_state
				
			# Update state errors
			update_state_errors()
		else:
			#print( "In manual mode." )
			pass
		Loop_rate.sleep()

