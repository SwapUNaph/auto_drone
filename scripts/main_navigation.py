#!/usr/bin/env python

# Script developed by Vincenz Frenzel (navigation by Derek Thompson)
#  --- Changelog ---
# Goal:     Main logic of autonomy
# Status:   07/11: Started with script
#           11/02: Added last comments to script

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
import common_resources as cr





def signal_handler(_, __):
	# enable Ctrl+C of the script
	sys.exit(0)



def navigate_gate():
	

	# navigation algorithm to fly us to WPs    
	bebop_p = bebop_model.pos
	hdg = bebop_model.att[2]
	global_vel = bebop_model.vel


	
	# rospy.loginfo("fly from")
	# rospy.loginfo([bebop_p[0], bebop_p[1], bebop_p[2], hdg])
	# rospy.loginfo("fly to")
	# rospy.loginfo(current_state.fly.pos)
	


	# global difference between WP ad position
	diff_global = current_state.fly.pos - bebop_p

	
	# X and Y components hypothenuse
	dist = math.hypot(diff_global[0], diff_global[1])


	# heading of the gate itself
	theta_gate = current_gate.hdg
	
	# heading from drone to gate position
	theta_pos = math.atan2(diff_global[1], diff_global[0])
	
	# difference between the two headings
	d_theta = cr.wrap2pi(theta_gate - theta_pos)
	
	

	# LATERAL CONTROLLER

	x_pos_proj = bebop_p[0] + global_vel[0] * point_time_proj
	y_pos_proj = bebop_p[1] + global_vel[1] * point_time_proj

	diff_global_proj = current_state.fly.pos - np.array([x_pos_proj,y_pos_proj,0.0])

	theta_pos_proj = math.atan2(diff_global_proj[1], diff_global_proj[0])

	dist_proj = math.hypot(diff_global_proj[0],diff_global_proj[1])

	d_theta_proj = cr.wrap2pi(theta_gate - theta_pos_proj)


	
	
	gate_nav_hold_dist = 2
	# gate_nav_theta_start = 20 * np.pi/180.0
	gate_nav_theta_thr = 10 * np.pi/180.0
	gate_nav_des_vel_lat = 1.5
	gate_nav_des_vel_nrm = 1.0
	
	vel_slope_gate_nrm = 0.8
	vel_slope_gate_lat = 1.5

	

	if abs(d_theta_proj) > gate_nav_theta_thr:
		desired_norm_pos = gate_nav_hold_dist
	else:
		desired_norm_pos = 0.0  


	# desired_norm_pos = gate_nav_hold_dist



	lat_error_proj = dist_proj * d_theta_proj       
	nrm_error_proj = desired_norm_pos - dist_proj


	nrm_vel_des = cr.limit_value(vel_slope_gate_nrm * (nrm_error_proj), gate_nav_des_vel_nrm)

	if dist_proj < gate_nav_hold_dist * 1.5 and abs(d_theta_proj) <= gate_nav_theta_thr:
		nrm_vel_des = nrm_vel_des * (gate_nav_theta_thr - abs(d_theta_proj)) / gate_nav_theta_thr


	lat_vel_des = cr.limit_value( vel_slope_gate_lat * (-lat_error_proj), gate_nav_des_vel_lat)
	
	



	x_vel_des_proj = -lat_vel_des * math.sin(theta_pos_proj) - nrm_vel_des * math.cos(theta_pos_proj)
	y_vel_des_proj = lat_vel_des * math.cos(theta_pos_proj) - nrm_vel_des * math.sin(theta_pos_proj)
	
	x_vel_error = x_vel_des_proj - global_vel[0]
	y_vel_error = y_vel_des_proj - global_vel[1]
	

	# min_arrive_time = .1
	# x_arrive_time = max(diff_global[0]/global_vel[0],min_arrive_time)
	# y_arrive_time = max(diff_global[1]/global_vel[1],min_arrive_time)
	x_arrive_time = point_time_proj
	y_arrive_time = point_time_proj

	x_accel_time = min(point_time_proj,x_arrive_time)
	y_accel_time = min(point_time_proj,y_arrive_time)
	
	#x_accel_des = x_vel_error/(x_accel_time) + x_vel_des * nav_drag    
	#y_accel_des = y_vel_error/(y_accel_time) + y_vel_des * nav_drag    
	x_accel_des = x_vel_error/(x_accel_time) + x_vel_des_proj * nav_drag    
	y_accel_des = y_vel_error/(y_accel_time) + y_vel_des_proj * nav_drag
	
	x_theta_des = np.arctan(x_accel_des/9.81)
	y_theta_des = np.arctan(y_accel_des/9.81)

	nav_cmd_x = (x_theta_des * 180 / np.pi) / 40
	nav_cmd_y = (y_theta_des * 180 / np.pi) / 40


	


	# limit commands
	nav_cmd_x_lim = cr.limit_value(nav_cmd_x, gate_nav_limit_lat)
	nav_cmd_y_lim = cr.limit_value(nav_cmd_y, gate_nav_limit_lat)


	# return navigation commands back into vehicle frame
	nav_cmd_x_veh = nav_cmd_x_lim * np.cos(-hdg) - nav_cmd_y_lim * np.sin(-hdg)
	nav_cmd_y_veh = nav_cmd_y_lim * np.cos(-hdg) + nav_cmd_x_lim * np.sin(-hdg)
	


	# Z CONTROLLER

	z_error = diff_global[2]
	nav_cmd_z = nav_PID_z.update(z_error)
	


	# R CONTROLLER
	r_error = cr.wrap2pi(theta_pos - hdg)
	
	nav_cmd_r = nav_PID_r.update(r_error)
	


	
	# Create twist msg
	msg = Twist()
	msg.linear.x = gate_lateral_gain*nav_cmd_x_veh
	msg.linear.y = gate_lateral_gain*nav_cmd_y_veh
	# msg.linear.x = 0.0
	# msg.linear.y = 0.0
	msg.linear.z = cr.limit_value(sum(nav_cmd_z), nav_limit_z)
	msg.angular.z = cr.limit_value(sum(nav_cmd_r), nav_limit_r)

	log_array = Float32MultiArray()
	
	log_array.data = [current_state.fly.pos[0],
		current_state.fly.pos[1],
		current_state.fly.pos[2],
		bebop_p[0],
		bebop_p[1],
		bebop_p[2],
		diff_global[0],
		diff_global[1],
		diff_global[2],
		x_pos_proj,
		y_pos_proj,
		dist,
		d_theta,
		diff_global_proj[0],
		diff_global_proj[1],
		d_theta_proj,
		lat_error_proj,
		nrm_error_proj,
		desired_norm_pos,
		lat_vel_des,
		nrm_vel_des,
		x_vel_des_proj,
		y_vel_des_proj,
		global_vel[0],
		global_vel[1],
		x_vel_error,
		y_vel_error,
		x_accel_time,
		x_accel_des,
		x_theta_des,
		nav_cmd_x,
		nav_cmd_x_lim,
		msg.linear.x,
		y_accel_time,
		y_accel_des,
		y_theta_des,
		nav_cmd_y,
		nav_cmd_y_lim,
		msg.linear.y,
		z_error,
		nav_cmd_z[0],
		nav_cmd_z[1],
		nav_cmd_z[2],
		sum(nav_cmd_z),
		msg.linear.z,
		theta_gate,
		theta_pos,
		hdg,
		r_error,
		nav_cmd_r[0],
		nav_cmd_r[1],
		nav_cmd_r[2],
		sum(nav_cmd_r),
		msg.angular.z]


	log_array.layout.dim.append(MultiArrayDimension())

	log_array.layout.dim[0].label = 'point_logs'
	log_array.layout.dim[0].size = len(log_array.data)



	publisher_nav_log.publish(log_array)
	return msg



def navigate_point():
	
	# navigation algorithm to fly us to WPs    
	bebop_p = bebop_model.pos

	hdg = bebop_model.att[2]
	
	# print current_state
	# print current_state.fly
	# print current_state.look

	target = current_state.fly.pos
	target_look = current_state.look.pos
	# transform velocities into global frame
	global_vel = bebop_model.vel

	
	diff_global = current_state.fly.pos - bebop_p
	

	# rospy.loginfo("fly from")
	# rospy.loginfo([bebop_p[0], bebop_p[1], bebop_p[2], hdg])
	# rospy.loginfo("fly to")
	# rospy.loginfo(current_state.fly.pos)
	
	
	# LATERAL CONTROLLER
	
	x_pos_error = diff_global[0]
	y_pos_error = diff_global[1]

	x_pos_proj = bebop_p[0] + global_vel[0] * point_time_proj
	y_pos_proj = bebop_p[1] + global_vel[1] * point_time_proj

	
	x_vel_des_proj = cr.limit_value(point_vel_slope*(target[0]-x_pos_proj),point_nav_des_vel)
	y_vel_des_proj = cr.limit_value(point_vel_slope*(target[1]-y_pos_proj),point_nav_des_vel)

	
	x_vel_error = x_vel_des_proj - global_vel[0]
	y_vel_error = y_vel_des_proj - global_vel[1]


	# min_arrive_time = .1
	# x_arrive_time = max(diff_global[0]/global_vel[0],min_arrive_time)
	# y_arrive_time = max(diff_global[1]/global_vel[1],min_arrive_time)
	x_arrive_time = point_time_proj
	y_arrive_time = point_time_proj

	x_accel_time = min(point_time_proj,x_arrive_time)
	y_accel_time = min(point_time_proj,y_arrive_time)
	
	#x_accel_des = x_vel_error/(x_accel_time) + x_vel_des * nav_drag    
	#y_accel_des = y_vel_error/(y_accel_time) + y_vel_des * nav_drag    
	x_accel_des = x_vel_error/(x_accel_time) + x_vel_des_proj * nav_drag    
	y_accel_des = y_vel_error/(y_accel_time) + y_vel_des_proj * nav_drag
	
	x_theta_des = np.arctan(x_accel_des/9.81)
	y_theta_des = np.arctan(y_accel_des/9.81)

	nav_cmd_x = (x_theta_des * 180 / np.pi) / 40
	nav_cmd_y = (y_theta_des * 180 / np.pi) / 40


	# limit commands
	nav_cmd_x_lim = cr.limit_value(nav_cmd_x, point_nav_limit_lat)
	nav_cmd_y_lim = cr.limit_value(nav_cmd_y, point_nav_limit_lat)

	
	# return navigation commands back into vehicle frame
	nav_cmd_x_veh = nav_cmd_x_lim * np.cos(-hdg) - nav_cmd_y_lim * np.sin(-hdg)
	nav_cmd_y_veh = nav_cmd_y_lim * np.cos(-hdg) + nav_cmd_x_lim * np.sin(-hdg)
	


	# Z CONTROLLER

	z_error = diff_global[2]
	nav_cmd_z = nav_PID_z.update(z_error)

	


	# R CONTROLLER
	
	diff_global_look = target_look - bebop_p
	pos_theta = np.arctan2(diff_global_look[1], diff_global_look[0])

	r_error = cr.wrap2pi(-(hdg - pos_theta))
	

	nav_cmd_r = nav_PID_r.update(r_error)
	


	
	# Create twist msg
	msg = Twist()
	msg.linear.x = point_lateral_gain*nav_cmd_x_veh
	msg.linear.y = point_lateral_gain*nav_cmd_y_veh
	msg.linear.z = cr.limit_value(sum(nav_cmd_z), nav_limit_z)
	msg.angular.z = cr.limit_value(sum(nav_cmd_r), nav_limit_r)


	log_array = Float32MultiArray()
	
	log_array.data = [current_state.fly.pos[0],
		current_state.fly.pos[1],
		current_state.fly.pos[2],
		x_pos_error,
		x_pos_proj,
		x_vel_des_proj,
		global_vel[0],
		x_vel_error,
		x_accel_time,
		x_accel_des,
		x_theta_des,
		nav_cmd_x,
		nav_cmd_x_lim,
		msg.linear.x,
		y_pos_error,
		y_pos_proj,
		y_vel_des_proj,
		global_vel[1],
		y_vel_error,
		y_accel_time,
		y_accel_des,
		y_theta_des,
		nav_cmd_y,
		nav_cmd_y_lim,
		msg.linear.y,
		z_error,
		nav_cmd_z[0],
		nav_cmd_z[1],
		nav_cmd_z[2],
		sum(nav_cmd_z),
		msg.linear.z,
		pos_theta,
		hdg,
		r_error,
		nav_cmd_r[0],
		nav_cmd_r[1],
		nav_cmd_r[2],
		sum(nav_cmd_r),
		msg.angular.z]


	log_array.layout.dim.append(MultiArrayDimension())

	log_array.layout.dim[0].label = 'point_logs'
	log_array.layout.dim[0].size = len(log_array.data)

	publisher_nav_log.publish(log_array)

	return msg



def calculate_distance():
	# calculate a distance based on which navigation is active

	global bebop_model
	
	# no waypoint or no odometry
	if current_state.fly is None or bebop_model.pos is None:
		return 999

	diff_global = current_state.fly.pos - bebop_model.pos

	if nav_active == "point":
		# use linear 3D distance between two points. Add penalty for angular offset
		linear_distance = np.linalg.norm(diff_global)

		return linear_distance 


	elif nav_active == "through":

		linear_distance = np.linalg.norm(diff_global)
		return linear_distance 
		
		# in any through navigation, use a plane instead that is parallel to gate itself. Eventually it doesn't matter
		# how far we are from the gate in 3D but how far we are from the plane of the gate
		flat_distance = np.linalg.norm([diff_global[0], diff_global[1], 0])
		heading_to_gate = math.atan2(current_state.fly.pos[1] - bebop_position.y, current_state.fly.pos[0] - bebop_position.x)
		heading_of_gate = current_state.fly.hdg
		heading_difference = heading_to_gate - heading_of_gate
		return flat_distance * math.cos(heading_difference)



def publish_status(st):
	pub = rospy.Publisher('/bebop/' + st, Empty, queue_size=1)
	if not rospy.is_shutdown():
		msg = Empty()
		pub.publish(msg)
	else:
		rospy.loginfo("status command not sent")



def callback_states_changed(data, args):
	# update states, either autonomy state or bebop state
	if args == "state_auto":
		global state_auto
		state_auto = data.data
		rospy.loginfo("state auto changed to " + str(state_auto))
	elif args == "state_bebop":
		global state_bebop
		state_bebop = data.state
		rospy.loginfo("state bebop changed to " + str(state_bebop))
	elif args == 'autonomy':
		global autonomy_active
		autonomy_active = data.data
		rospy.loginfo("autonomy active changed to " + str(autonomy_active))



def callback_visual_gate_detection_changed(data):
	
	# gate_position, gate_hdg = cr.WP2array(data)
	

	global bebop_model
	global nav_active
	global detection_active
	global current_gate
	global gate_last_meas_time
	global gate_detected
	global gate_log
	global std_gate_thres
	

	if not detection_active:
		rospy.loginfo('Detection not supposed to be active, exiting visual_callback')
		return
	
	if len(data.format) == 0:
		pass


	if data.format != 'no_gate' and data.format != 'gate':

		

		if current_gate is not None:
			
			# print current_gate
			# print current_gate.format
			# print current_gate.on_gate

			if current_gate.on_gate is None:

				if data.format == 'left' and current_gate.format == 'horizontal':
					current_gate.update_format('left')
					bebop_model.update_frame_change(np.array([-.7, 0., 0.]))
				
				elif data.format == 'right' and current_gate.format == 'horizontal':
					current_gate.update_format('right')
					bebop_model.update_frame_change(np.array([.7, 0., 0.]))

				
				elif data.format == 'up' and current_gate.format == 'vertical':
					current_gate.update_format('up')
					bebop_model.update_frame_change(np.array([0., 0., 0.7]))
				

				elif data.format == 'down' and current_gate.format == 'vertical':
					current_gate.update_format('down')
					bebop_model.update_frame_change(np.array([0., 0., -0.7]))

				else:
					rospy.loginfo('Error: couldnt match gate format')

		else:
			rospy.loginfo('Error: current_gate not initialized correctly')
	
	

	if gate_detected is not None:

		if data.format == 'no_gate':
			gate_detected = False
		else:
			gate_detected = True
			gate_last_meas_time = time.time()


		if nav_active == 'through' and gate_detected:
			rospy.loginfo('Updating pose from gate: '+str(math.hypot(data.pos.x,data.pos.y))+', '+str(data.pos.z)+', '+str(data.format))
			
			meas_vec = np.array([data.pos.x, data.pos.y, data.pos.z])
			gate_pos = current_gate.pos.pos
			R_body2track = tfs.rotation_matrix(bebop_model.att[2],(0,0,1))[0:3,0:3]
			gate_proj = np.matmul(R_body2track, meas_vec)
			pos_calc = gate_pos - gate_proj

			gate_log.append(pos_calc)

	else:
		
		meas_vec = np.array([data.pos.x, data.pos.y, data.pos.z])
		R_body2track = tfs.rotation_matrix(bebop_model.att[2],(0,0,1))[0:3,0:3]
		gate_proj = np.matmul(R_body2track, meas_vec)
		pos_calc = bebop_model.pos + gate_proj
		

		if len(gate_log) < 10:
			gate_log.append(pos_calc)

		else:

			
			gate_log.pop(0)
			gate_log.append(pos_calc)

			gate_log_np = np.array(gate_log)
			
			std_x = np.std(gate_log_np[:,0:1])
			std_y = np.std(gate_log_np[:,1:2])
			std_z = np.std(gate_log_np[:,2:3])
			std_pos = np.linalg.norm(np.array([std_x, std_y, std_z]))
			
			if std_pos < std_gate_thres:
				rospy.loginfo('Std below threshold: '+str(std_pos))



				# Estimate current gate position based on first estimated position
				global estimate_vertical_gate
				global estimate_horizontal_gate
				
				if current_gate.format == 'vertical' and estimate_vertical_gate:
					
					if np.mean(gate_log_np[:,2:3]) > current_gate.pos.pos[2]:
						current_gate.update_format('up')
					else:
						current_gate.update_format('down')


				if current_gate.format == 'horizontal' and estimate_horizontal_gate:

					if np.mean(gate_log_np[:,0:1]) < current_gate.pos.pos[0]:
						current_gate.update_format('left')
					else:
						current_gate.update_format('right')



				gate_pos = current_gate.pos.pos
				pos_calc = gate_pos - gate_proj


				gate_log = [pos_calc]
				gate_detected = True

				
			else:
				rospy.loginfo('Measured Std too high: '+str(std_pos))


			log_array = Float32MultiArray()
			
			
			log_array.data = [meas_vec[0],
							  meas_vec[1],
							  meas_vec[2],
							  pos_calc[0],
							  pos_calc[1],
							  pos_calc[2],
							  len(gate_log),
							  std_x,
							  std_y,
							  std_z,
							  std_pos]


			log_array.layout.dim.append(MultiArrayDimension())

			log_array.layout.dim[0].label = 'gate_logs'
			log_array.layout.dim[0].size = len(log_array.data)


			publisher_visual_log.publish(log_array)
		


def callback_bebop_odometry_changed(data):
	# main loop, every time odometry changes
	# global bebop_odometry
	bebop_pose = data.pose.pose.position
	bebop_p = np.array([bebop_pose.x, bebop_pose.y, bebop_pose.z])
	quat = data.pose.pose.orientation
	bebop_hdg = tfs.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]
	rospy.loginfo("Odom recieved: " + str([bebop_pose.x, bebop_pose.y, bebop_pose.z, bebop_hdg]))


	global bebop_model
	global nav_active
	global publisher_model
	global publisher_visual_log
	global bebop_odometry
	global bebop_last_known_pos
	global track_last_known_pos
	global track_last_known_hdg
	global bebop_last_known_hdg
	global bebop_last_odom_time
	global gate_gain_comp_filter
	global gate_log
	global last_track_pos

	# Update body velocity
	bebop_model.update_body_vel(data.twist.twist.linear)


	# if there is no odometry, don't do anything
	if bebop_odometry is None:
		rospy.loginfo("No position, publish empty driving msg")
		send_bebop_cmd(Twist())
		
		bebop_last_odom_time = time.time()
		bebop_odometry = data
		return

	if current_state is None:
		rospy.loginfo("No State, exiting odom_callback")
		send_bebop_cmd(Twist())

		return 
		# bebop_last_odom_time = time.time()
		# bebop_odometry = data

		# bebop_odom_transformed = bebop2track_transform_odom(data)
		# bebop_model.update_odom(bebop_odom_transformed)

		

	# if there is no track frame reference point, set current position to takeoff 
	if bebop_last_known_pos is None:
		update_last_known_bebop(data)

	



	# if point nav or nav off update pose to track frame and update
	if nav_active == "off" or  nav_active == "point":
		bebop_odom_transformed = bebop2track_transform_odom(data)
		bebop_model.update_odom(bebop_odom_transformed)

		


	elif nav_active == 'through':

		bebop_odom_transformed = bebop2track_transform_odom(data)
		
		bebop_model.update_orientation(bebop_odom_transformed)
		
		pos_c = bebop_odom_transformed.pose.pose.position

		d_pos_odom = np.array([pos_c.x, pos_c.y, pos_c.z]) - last_track_pos

		if len(gate_log) > 0:

			np_log = np.array(gate_log)
			x_avg = np.mean(np_log[:,0])
			y_avg = np.mean(np_log[:,1])
			z_avg = np.mean(np_log[:,2])
			avg = [x_avg, y_avg, z_avg]

			gate_log = []
		
			d_pos_gate = avg - last_track_pos

			d_pos_filtered = (1 - gate_gain_comp_filter) * d_pos_odom + (gate_gain_comp_filter) * d_pos_gate
			
			# print d_pos_filtered

			bebop_model.update_pose_gate(d_pos_filtered)

			track_last_known_pos = np.copy(bebop_model.pos)
			bebop_last_known_pos = bebop_p


			log_array = Float32MultiArray()
			
			
			log_array.data = [x_avg,
							  bebop_model.pos[0],
							  y_avg,
							  bebop_model.pos[1],
							  z_avg,
							  bebop_model.pos[2],
							  bebop_last_known_pos[0],
							  bebop_last_known_pos[1],
							  bebop_last_known_pos[2],
							  track_last_known_pos[0],
							  track_last_known_pos[1],
							  track_last_known_pos[2],
							  bebop_last_known_hdg,
							  track_last_known_hdg,
							  d_pos_filtered[0],
							  d_pos_filtered[1],
							  d_pos_filtered[2],
							  bebop_hdg,
							  bebop_model.att[2]]


			log_array.layout.dim.append(MultiArrayDimension())

			log_array.layout.dim[0].label = 'gate_logs'
			log_array.layout.dim[0].size = len(log_array.data)


			publisher_visual_log.publish(log_array)


		else:
			
			bebop_model.update_pose_gate(d_pos_odom)




	last_track_pos = np.copy(bebop_model.pos)

	# Send out bebops estimated position
	publisher_model.publish(bebop_model.get_drone_pose())

	bebop_odometry = data

	bebop_last_odom_time = time.time()
	
	
	
def update_last_known_bebop(odom):
	
	global bebop_last_known_hdg
	global bebop_last_known_pos
	global track_last_known_pos
	global last_track_pos

	
	pos = odom.pose.pose.position
	quat = odom.pose.pose.orientation

	bebop_last_known_pos = np.array([pos.x, pos.y, pos.z])
	bebop_last_known_hdg = tfs.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]

	track_last_known_pos[2] = pos.z
	last_track_pos = np.copy(track_last_known_pos)



def bebop2track_transform_odom(bebop_odom):

	global track_last_known_pos
	global bebop_last_known_pos
	global bebop_last_known_hdg
	global track_last_known_hdg


	pose = bebop_odom.pose.pose
	twist = bebop_odom.twist.twist.linear

	bebop_d_pos = np.array([pose.position.x, pose.position.y, pose.position.z]) - bebop_last_known_pos
	bebop_q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])


	hdg_bebop = tfs.euler_from_quaternion(bebop_q)[2]
	track_bebop_dhdg =  track_last_known_hdg - bebop_last_known_hdg

	R_hdg = tfs.rotation_matrix(track_bebop_dhdg,(0,0,1))[0:3,0:3]

	track_pos = np.matmul(R_hdg,bebop_d_pos) + track_last_known_pos

	bebop_vel_body = np.array([twist.x,twist.y,twist.z])

	R_body2track = tfs.rotation_matrix(hdg_bebop+track_bebop_dhdg,(0,0,1))[0:3,0:3]

	track_vel_body = np.matmul(R_body2track,bebop_vel_body)

	track_odom = bebop_odom

	track_odom.twist.twist.linear.x = track_vel_body[0]
	track_odom.twist.twist.linear.y = track_vel_body[1]
	track_odom.twist.twist.linear.z = track_vel_body[2]
		
	track_odom.pose.pose.position.x = track_pos[0]
	track_odom.pose.pose.position.y = track_pos[1]
	track_odom.pose.pose.position.z = track_pos[2]

	track_quat = tfs.quaternion_multiply(tfs.quaternion_from_euler(0,0,track_bebop_dhdg),bebop_q)

	track_odom.pose.pose.orientation.x = track_quat[0]
	track_odom.pose.pose.orientation.y = track_quat[1]
	track_odom.pose.pose.orientation.z = track_quat[2]
	track_odom.pose.pose.orientation.w = track_quat[3]
	
	return track_odom



def update_pose_estimate():
	#global nav_active
	#global bebop_model
	
	# rospy.loginfo("Updating and sending Pose")
	
	# if nav_active == 'point' or  nav_active == 'off':
	#     if bebop_last_known_pos is not None:
	bebop_model.propagate(1.0/loop_rate)
	
	temp_pose = bebop_model.get_drone_pose()
	temp_odom = Odometry()
	temp_odom.pose.pose = temp_pose

	publisher_model.publish(temp_pose)
	# publisher_model_odom.publish(temp_odom)
	
	# calculate distance from WP
	navigation_distance = calculate_distance()
	rospy.loginfo("navigation distance: "+str(navigation_distance))

	# state_machine_advancement (if conditions are met: distances, states, ...)
	current_state.check(navigation_distance)



def send_bebop_cmd(twist):
	global autonomy_active

	if autonomy_active:
		publisher_auto_drive.publish(twist)



def update_controller():
	
	if nav_active == "off":
		# navigation off, send empty message and return
		rospy.loginfo("Navigation turned off")
		#rospy.loginfo("publish empty driving msg")
		send_bebop_cmd(Twist())


	elif nav_active == "point":
		if bebop_last_known_pos is not None:
			auto_driving_msg = navigate_point()
			send_bebop_cmd(auto_driving_msg)
	

	elif nav_active == "through":
		auto_driving_msg = navigate_gate()
		send_bebop_cmd(auto_driving_msg)


	elif nav_active == 'bebop':
		if current_state.condition_thres == cr.Bebop.HOVERING:
			publish_status('takeoff')
		elif current_state.condition_thres == cr.Bebop.LANDED:
			publish_status('land')
		else:
			send_bebop_cmd(Twist())



class State:
	# This is the ominous state machine
	def __init__(self, own_state=None, next_state=None, condition_type=None, condition_thres=None,
				 exit_clear_visual=None, exit_clear_gate=None, detection_active_bool=None, choose_active_bool=None, nav_active_str=None,
				 gate_color=None, fly=None, look=None, gate_current=None):
		self.own_state = own_state                              # own state id
		self.next_state = next_state                            # next state id
		self.condition_type = condition_type                    # type of state advancement
		self.condition_thres = condition_thres                  # value for state advancement (not req. for all)
		self.exit_clear_visual = bool(exit_clear_visual)        # on state exit, clear all visual data? (new gate)
		self.exit_clear_gate = bool(exit_clear_gate)            # on state exit, reset current gate
		self.detection_active = bool(detection_active_bool)     # detection active?
		self.choose_active = bool(choose_active_bool)     		# detection active?
		self.nav_active = nav_active_str                        # which navigation should be active
		self.gate_color = gate_color                            # color of the gate
		self.fly = fly                                          # if blind waypoint should be calculated: offset for
		self.look = look                                        # flying and looking waypoint into gate direction
		self.time = None                                        # timer for takeoff wait
		self.current_gate = gate_current                        # on enter, set current_gate to the correct gate class variable


	def enter(self):
		# do things when state is selected
		print("enter state " + str(self.own_state))
		# change global state id and also publish state id
		global current_state
		current_state = states[self.own_state]
		publisher_state_auto.publish(self.own_state)

		# reset all PID loops
		nav_PID_z.reset()
		nav_PID_r.reset()

		# if timer is required for condition, start timer
		self.time = time.time()


		# # if gate color is defined, publish gate color
		if self.gate_color is not None:
			msg = Float32MultiArray()
			msg.data = self.gate_color
			publisher_gate_color.publish(msg)
		
		# set applicable navigation
		global nav_active
		nav_active = self.nav_active

		

		# set detection to on/off
		global detection_active
		global estimate_vertical_gate
		global estimate_horizontal_gate
		
		detection_active = self.detection_active

		detection_msg = Detection_Active()
		detection_msg.active.data = self.detection_active

		if self.detection_active:
			detection_msg.vertical.data = bool(self.current_gate.format == 'vertical')
			detection_msg.choose.data = self.choose_active
			
		publisher_gate_detection.publish(detection_msg)



		# Initialize current gate to gate found
		if self.current_gate is not None:
			global current_gate
			current_gate = self.current_gate


		# publish blind WP for ground control station
		if self.look is not None:
			msg = WP_Msg()    
			msg.pos.x = self.look.pos[0]
			msg.pos.y = self.look.pos[1]
			msg.pos.z = self.look.pos[2]
			msg.hdg = 0
			global publisher_wp_look 
			publisher_wp_look.publish(msg)

		
		# publish look WP for ground control station
		if self.fly is not None:
			msg = WP_Msg()
			msg.pos.x = self.fly.pos[0]
			msg.pos.y = self.fly.pos[1]
			msg.pos.z = self.fly.pos[2]
			if self.fly.hdg is not None:
				msg.hdg = self.fly.hdg

			global publisher_wp_fly 
			publisher_wp_fly.publish(msg)



		
	def exit(self):
		# do things when state is finished
		print("exit state " + str(self.own_state))

		# if visuals should be cleared, do it now (preparation for next gate)
		if self.exit_clear_visual:
			pass

		if self.exit_clear_gate:
			global gate_last_meas_time
			global current_gate
			global gate_detected

			gate_detected = None
			gate_last_meas_time = None
			current_gate.reset()

		# enter new state
		if self.next_state != 99:
			states[self.next_state].enter()


	def check(self, navigation_distance):
		# setup state advancement condition
		rospy.loginfo("state " + str(state_auto) + ' ,Nav: ' + nav_active)

		# distance: if distance is smaller than value (could be scaled if there is a scale)
		if self.condition_type == "dist":
			if navigation_distance < self.condition_thres:
				self.exit()

		# waypoint: if average waypoint is found (not None anymore). visual_wp is "locked on"
		elif self.condition_type == "wp":
			if gate_detected is not None:
				self.exit()

		# specific condition of bebop state machine (hover, land, ...)
		elif self.condition_type == "bebop":
			if state_bebop == self.condition_thres:
				self.exit()

		# some time has elapsed (on takeoff). Required because magnetometer changes a lot while motors spin up and drone
		# takes off
		elif self.condition_type == "time":
			if time.time() > self.time + self.condition_thres:
				self.exit()



def emergency_shutdown(_):
	# if this is triggered, it shuts off node for performance reasons
	rospy.loginfo("emergency shutdown")
	# rospy.signal_shutdown("emergency shutdown")



if __name__ == '__main__':
	# Enable killing the script with Ctrl+C.
	signal.signal(signal.SIGINT, signal_handler)

	# initialize node
	rospy.init_node('main_navigation', anonymous=False)

	# Variables
	autonomy_active = False                                     # autonomous mode is active
	

	# Competition
	# Gate 1
	# start_pos = np.array([-.7, 4.0, 0.])
	# start_hdg = -np.pi/2

	# Gate 2
	start_pos = np.array([1.4, 6.0, 0.])
	start_hdg = np.pi/2


	# Mini course
	# start_pos = np.array([-.7, 3.5, 0.])
	# start_hdg = -np.pi/2

	# Testing
	# start_pos = np.array([0.0, 0.0, 0.0])
	# start_hdg = 0.0

	

	bebop_model = cr.Bebop_Model(start_pos,start_hdg)           # Initialize odometry message to store position




	bebop_odometry = None                                       # latest odometry from bebop
	bebop_last_known_pos = None                                 # keep track of last known position of bebop for tranform to track frame
	bebop_last_known_hdg = None
	track_last_known_pos = np.copy(start_pos)
	track_last_known_hdg = start_hdg

	state_auto = 0                                              # initialize state machine (number of state)
	current_state = None                                        # the actual state object with method "check"
	state_bebop = None                                          # state of drone itself (own state machine)
	loop_rate = 20                                              # frequency model and controller are run at

	current_gate = None
	gate_log = []                                               # Filtered gate input from gate detection
	gate_detected = None
	gate_gain_comp_filter = .2                                  # input of the gate to the complementary filter
	std_gate_thres = 1.4

	estimate_vertical_gate = True
	estimate_horizontal_gate = True


	bebop_last_odom_time = None                                 # Time log of the last bebop_odom data
	gate_last_meas_time = None                                  # Time log of the last gate data
	
	
	detection_active = False                                    # gate detection active boolean
	nav_active = "off"                                          # activated navigation algorithm
	last_track_pos = start_pos
	
	# PID Controller Classes
	nav_PID_z = cr.PID(1.0, 0, 0.0)
	nav_PID_r = cr.PID(1.0, 0, 0.1)



	# Controller Params
	
	point_time_proj = .5                                        # acceleration response time 
	point_nav_des_vel = 1.5                                     # max calculated velocity
	point_vel_slope = 2.0                                       # Desired vel slope
	point_lateral_gain = 1.0                                    # Lateral command gain
	point_nav_limit_lat = .2                                    # lateral command limit

	
	gate_nav_hold_dist = 2.5
	gate_nav_theta_thr = 15 * np.pi/180.0
	gate_nav_des_vel_lat = 2.0
	gate_nav_des_vel_nrm = 1.5
	gate_lateral_gain = 1.0
	gate_nav_limit_lat = .3                                      # absolute limit to all cmd before being sent, thr_nav
	vel_slope_gate_nrm = 1.2
	vel_slope_gate_lat = 1.5




	nav_limit_z = .2                                            # Z command limit
	nav_limit_r = .8                                            # R command limit
	
	nav_drag = 0.0                                              # Drag acceleration term


	

	# Publishers
	publisher_state_auto = rospy.Publisher(         "/auto/state_auto",         Int32,                  queue_size=1, latch=True)
	publisher_auto_drive = rospy.Publisher(         "/bebop/cmd_vel",           Twist,                  queue_size=1, latch=True)
	publisher_model = rospy.Publisher(              "/auto/pose",               Drone_Pose,             queue_size=1)
	publisher_model_odom = rospy.Publisher(         "/auto/pose_odom",          Odometry,               queue_size=1)

	publisher_wp_fly = rospy.Publisher(             "/auto/wp_fly",             WP_Msg,                 queue_size=1)
	publisher_wp_look = rospy.Publisher(            "/auto/wp_look",            WP_Msg,                 queue_size=1)

	publisher_wp_current = rospy.Publisher(         "/auto/wp_current",         WP_Msg,                 queue_size=1, latch=True)
	publisher_nav_log = rospy.Publisher(            "/auto/navigation_logger",  Float32MultiArray,      queue_size=1, latch=True)
	publisher_visual_log = rospy.Publisher(         "/auto/visual_logger",      Float32MultiArray,      queue_size=1, latch=True)

	publisher_gate_color = rospy.Publisher(         "/auto/gate_color",         Float32MultiArray,      queue_size=1, latch=True)
	publisher_gate_detection = rospy.Publisher(  "/auto/gate_detection_active", Detection_Active,       queue_size=1, latch=True)

	
	# abbreviations for state machine setup
	p = "point"
	t = "through"
	o = "off"
	b = "bebop"

	# Params if needs to guess gate
	# 0 No gate detection
	v = 'vertical' # Gate detection vertical
	h = 'horizontal' # Gate detection vertical




	gate_look_dist = 4.0
	gate_exit_dist = 2.0


	dist_gate_blind = 1.0                                       # how exact go to blind wp before advancing state
	dist_gate_close = 1.5                                       # how close to gate before advancing state (approach)
	exit_thrs = gate_exit_dist - .75                            # how far behind the gate is it considered passed





	# Madrid color values for gates
	gate_params1 = np.array([100, 140, 50, 140, 255, 255])
	gate_params2 = np.array([100, 140, 50, 140, 255, 255])



	# Navigation Waypoints

	start_wp = cr.WP(np.array(start_pos),None)

	turn_pos_1 = cr.WP(np.array([2.0, -1.25, 1.7]),None)
	turn_pos_2 = cr.WP(np.array([-1.5, 11.25, 1.7]),None)
	
	gate_1 = cr.Gate(v,np.array([-0.7, 0.0, 2.2]), -np.pi/2, gate_look_dist, gate_exit_dist)
	gate_2 = cr.Gate(h,np.array([1.4, 10.0, 1.7]), np.pi/2, gate_look_dist, gate_exit_dist)
	


	# Test stuff

	pnt1 = cr.WP(start_pos+[0, 0, 1.7],None)
	pnt1 = cr.WP(start_pos+[3, 0, 1.7],None)
	look = cr.WP(start_pos+[7, 0, 1.7],None)
	

	
	# Small Test track

	gate_1_test = cr.Gate(v,np.array([-0.7, 0.0, 2.5]),-np.pi/2, 3.5, 2.0)
	gate_2_test = cr.Gate(v,np.array([1.4, 3.2, 2.5]),np.pi/2, 3.5, 2.25)
	

	turn_pos_1_test = cr.WP(np.array([1.4, -.75, 1.8]),None)
	turn_pos_2_test = cr.WP(np.array([-0.7, 4.25, 1.8]),None)
	
	



	# STATE SETUP

	init_state = 20

	# own_state, next_state, condition_type, condition_thres, exit_clear_visual, reset_gate, detection_active_type, nav_active_str, gate_color, fly, look, gate)
	
	states = [State()] * 100

	# states[02] = State(02, 03, "bebop", cr.Bebop.TAKEOFF,  0, 0, 0, b, None, None, None, None)                                                        # Landed
	states[03] = State(03, 04, "bebop", cr.Bebop.HOVERING, 0, 0, 0, 0, b, None, None, None, None)                                                          # Taking off
	states[04] = State(04, 10, "time",  1.0,               0, 0, 0, 0, o, None, None, None, None)                                                          # Hovering


	
	# Forward and backwards testing
	# states[10]  = State(10, 11,  "dist",  1.0,            0, 0, 0, t, None,   gate_1.pos,        gate_1.pos,        gate_1)                               # testing 
	# states[11]  = State(11, 10, "dist",  1.0,            0, 0, 0, p, None,   gate_1.exit_pos,   gate_1.exit_pos,   None)                                 # testing 


	# One gate pass testings
	states[13] = State(13, 14, "dist",  dist_gate_blind,   0, 0, 0, 0, p, None,           gate_1.look_pos,        gate_1.pos,         None)                # Move and Look for gate
	states[14] = State(14, 15, "wp",    None,              0, 0, 1, 0, p, gate_params1,   gate_1.look_pos,        gate_1.pos,         gate_1)              # Move and Look for gate
	states[15] = State(15, 16, "dist",  dist_gate_close,   1, 0, 1, 0, t, gate_params1,   gate_1.pos,             gate_1.pos,         gate_1)              # move through gate until cant see it
	states[16] = State(16, 98, "dist",  exit_thrs,         0, 1, 0, 0, p, None,           gate_1.exit_pos,        gate_1.exit_pos,    None)                # Move through gate
	states[17] = State(17, 98, "dist",  dist_gate_blind,   0, 0, 0, 0, p, None,           turn_pos_1,             gate_2.pos,         None)                # Turn around manuever

	
	# Gate 2 Pass
	states[20] = State(20, 21, "dist",  dist_gate_blind,   0, 0, 0, 0, p, None,           gate_2.look_pos,        gate_2.pos,         None)                # Move and Look for gate
	states[21] = State(21, 22, "wp",    None,              0, 0, 1, 0, p, gate_params1,   gate_2.look_pos,        gate_2.pos,         gate_2)              # Move and Look for gate
	states[22] = State(22, 23, "dist",  dist_gate_close,   1, 0, 1, 0, t, gate_params1,   gate_2.pos,             gate_2.pos,         gate_2)              # move through gate until cant see it
	states[23] = State(23, 98, "dist",  exit_thrs,         0, 1, 0, 0, p, None,           gate_2.exit_pos,        gate_2.exit_pos,    None)                # Move through gate
	



	# Mini course
	states[29] = State(29, 30, "dist",  dist_gate_blind,   0, 0, 0, 0, p, None,           gate_1_test.look_pos,        gate_1_test.pos,         None)                # Move and Look for gate
	
	states[30] = State(30, 31, "wp",    None,              0, 0, 1, 0, p, gate_params1,   gate_1_test.look_pos,        gate_1_test.pos,         gate_1_test)              # Move and Look for gate
	states[31] = State(31, 32, "dist",  dist_gate_close,   1, 0, 1, 0, t, gate_params1,   gate_1_test.pos,             gate_1_test.pos,         gate_1_test)              # move through gate until cant see it
	states[32] = State(32, 33, "dist",  exit_thrs,         0, 1, 0, 0, p, None,           gate_1_test.exit_pos,        gate_1_test.exit_pos,    None)                # Move through gate
	states[33] = State(33, 40, "dist",  dist_gate_blind,   0, 0, 0, 0, p, None,           turn_pos_1_test,             gate_2_test.pos,         None)                # Turn around manuever


	states[40] = State(40, 41, "wp",    None,              0, 0, 1, 0, p, gate_params2,   gate_2_test.look_pos,        gate_2_test.pos,         gate_2_test)              # Move and Look for gate
	states[41] = State(41, 42, "dist",  dist_gate_close,   1, 0, 1, 0, t, gate_params2,   gate_2_test.pos,             gate_2_test.pos,         gate_2_test)              # move through gate until cant see it
	states[42] = State(42, 43, "dist",  exit_thrs,         0, 1, 0, 0, p, None,           gate_2_test.exit_pos,        gate_2_test.exit_pos,    None)                # Move through gate
	states[43] = State(43, 30, "dist",  dist_gate_blind,   0, 0, 0, 0, p, None,           turn_pos_2_test,             gate_1_test.pos,         None)                # Turn around manuever
	


	
	# Full Gate test passes
	states[49] = State(29, 30, "dist",  dist_gate_blind,   0, 0, 0, 0, p, None,           gate_1_test.look_pos,			gate_1.pos,         None)                # Move and Look for gate
	
	states[50] = State(50, 51, "wp",    None,              0, 0, 1, 0, p, gate_params1,   gate_1_test.look_pos,			gate_1.pos,         gate_1)              # Move and Look for gate
	states[51] = State(51, 52, "dist",  dist_gate_close,   1, 0, 1, 0, t, gate_params1,   gate_1_test.pos,				gate_1.pos,         gate_1)              # move through gate until cant see it
	states[52] = State(52, 53, "dist",  exit_thrs,         0, 1, 0, 0, p, None,           gate_1.exit_pos,        		gate_1.exit_pos,    None)                # Move through gate
	states[53] = State(53, 60, "dist",  dist_gate_blind,   0, 0, 0, 0, p, None,           turn_pos_1,             		gate_2.pos,         None)                # Turn around manuever


	states[60] = State(60, 61, "wp",    None,              0, 0, 1, 0, p, gate_params2,   gate_2.look_pos,        gate_2.pos,         gate_2)              # Move and Look for gate
	states[61] = State(61, 62, "dist",  dist_gate_close,   1, 0, 1, 1, t, gate_params2,   gate_2.pos,             gate_2.pos,         gate_2)              # move through gate until cant see it
	states[62] = State(62, 63, "dist",  exit_thrs,         0, 1, 0, 0, p, None,           gate_2.exit_pos,        gate_2.exit_pos,    None)                # Move through gate
	states[63] = State(63, 50, "dist",  dist_gate_blind,   0, 0, 0, 0, p, None,           turn_pos_2,             gate_1.pos,         None)                # Turn around manuever




	# End states
	states[98] = State(98, 99, "bebop", 	cr.Bebop.LANDED,   0, 0, 0, 0, o, None, None, None, None)      # landed
	states[99] = State(99, 99, "time", 				 1000.0,   0, 0, 0, 0, o, None, None, None, None)      # end state
	




	# Subscribers
	rospy.Subscriber("/auto/state_auto",            Int32,          callback_states_changed, "state_auto")
	rospy.Subscriber("/auto/autonomy_active",       Bool,           callback_states_changed, "autonomy")
	rospy.Subscriber("/bebop/odom",                 Odometry,       callback_bebop_odometry_changed)
	rospy.Subscriber("/auto/filtered_gate_WP",      WP_Msg,         callback_visual_gate_detection_changed)
	rospy.Subscriber("/auto/emergency_shutdown",    Empty,          emergency_shutdown)
	rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged,callback_states_changed, "state_bebop")
	




	# initializes startup by publishing state 0
	publisher_state_auto.publish(0)

	
	while bebop_odometry is None:
		time.sleep(.25)


	rospy.loginfo("Odometry recieved")
	update_last_known_bebop(bebop_odometry)


	rospy.loginfo("Waiting for autonomy")
	while not autonomy_active:
		time.sleep(.25)
	
		
	# enter state init_state in state machine
	rospy.loginfo("Autonomy active, starting states")
	publisher_state_auto.publish(init_state)
	states[init_state].enter()

	
	
	rate = rospy.Rate(loop_rate)
	while not rospy.is_shutdown():
		
		if autonomy_active:
			rospy.loginfo('Controller Loop')
			update_pose_estimate()
			update_controller()
		else:
			pass

		rate.sleep()
		

	# rospy.spin()
