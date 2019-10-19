#!/usr/bin/env python

# Script developed by Vincenz Frenzel (PIDs by Derek Thompson, frequency extraction by Matt Vaughn,
#					orientation conversions, Kalman Filter, Extended Kalman Filter and MVA filter by Swapneel Naphade)
#  --- Changelog ---
# Goal:     Collect information that is needed at multiple locations of other scripts or that are less essential
# Status:   11/03/18: Started changelog and commented code
#			7/9/19: Added Moving average filter, orientation conversions

import math
from tf import transformations as tfs
from geometry_msgs.msg import Twist, Pose, Quaternion, Point
from auto_drone.msg import WP_Msg, Drone_Pose, Detection_Active
import numpy as np

# Convert WP_Msg to array
def WP2array(wp):
	return (np.array([wp.pos.x, wp.pos.y, wp.pos.z]), float(wp.hdg), str(wp.format))
	
# Convert arrays to WP_Msg
def array2WP(pos, hdg, formt):
	wp = WP_Msg()
	point = Point()
	point.x = pos[0]
	point.y = pos[1]
	point.z = pos[2]
	wp.pos = point
	wp.hdg = float(hdg)
	wp.format = str(formt)
	return wp

# Convert Pose to a tuple of numpy array (position, orientation)
def pose2array(pose):
	return (np.array([pose.position.x, pose.position.y, pose.position.z]), np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]))

# Convert numpy arrays (position, orientation) to Pose
def array2pose(position, orientation):
	pose = Pose()
	pose.position.x = position[0]
	pose.position.y = position[1]
	pose.position.z = position[2]
	pose.orientation.w = orientation[3]
	pose.orientation.x = orientation[0]
	pose.orientation.y = orientation[1]
	pose.orientation.z = orientation[2]
	return pose

# Calculate Pose difference -> Vector diff. for position and quaternion diff. for orientation
def pose_diff(pose1, pose2):
	pose1_pos, pose1_orien = pose2array(pose1)
	pose2_pos, pose2_orien = pose2array(pose2)	
	pose_diff_pos = pose1_pos - pose2_pos
	pose_diff_orien = tfs.quaternion_multiply( pose1_orien, tfs.quaternion_conjugate(pose2_orien)  )	
	return array2pose(pose_diff_pos, pose_diff_orien)

# Convert Twist to a tuple of numpy array (linear, angular)	
def twist2array(twist):
	return (np.array([twist.linear.x, twist.linear.y, twist.linear.z]), np.array([twist.angular.x, twist.angular.y, twist.angular.z]))

# Convert numpy arrays (linear, angular) to Twist
def array2twist(linear, angular):
	twist = Twist()
	twist.linear.x = linear[0]
	twist.linear.y = linear[1]
	twist.linear.z = linear[2]
	twist.angular.x = angular[0]
	twist.angular.y = angular[1]
	twist.angular.z = angular[2]
	return twist

# Calculate Twist difference -> Vector diff. for linear and angular	
def twist_diff(twist1, twist2):
	twist_lin1, twist_ang1 = twist2array(twist1)
	twist_lin2, twist_ang2 = twist2array(twist2)	
	return array2twist(twist_lin1 - twist_lin2, twist_ang1 - twist_ang2)

# vector and rotation from bebop to zed
BZ = np.array([0, 0, 0])
zRb = tfs.euler_matrix(-math.pi / 2, 0, -math.pi / 2, 'rzyx')
cam_q = tfs.quaternion_from_matrix(zRb)
zRb = zRb[:3, :3]


# Convert quaternion to euler angles (angles in radians)
def quat2euler(q):
	 #q = [x,y,z,w]
	roll = np.arctan2( 2*(q[3]*q[0] + q[1]*q[2]), 1 - 2*(q[0]**2 + q[1]**2) )
	pitch = np.arcsin( 2*(q[3]*q[1] - q[0]*q[2]) )
	yaw = np.arctan2( 2*(q[3]*q[2] + q[0]*q[1]), 1 - 2*(q[1]**2 + q[2]**2) )
	return np.array([roll,pitch,yaw])
	
# Convert euler angles to quaternion (angles in radians)
def euler2quat(euler): # euler = [roll (X), pitch (Y), yaw (Z)]
	# Abbreviations for the various angular functions
	cy = math.cos(euler[2] * 0.5)
	sy = math.sin(euler[2] * 0.5)
	cp = math.cos(euler[1] * 0.5)
	sp = math.sin(euler[1] * 0.5)
	cr = math.cos(euler[0] * 0.5)
	sr = math.sin(euler[0] * 0.5)

	q = np.zeros(4)
	q[3] = cy * cp * cr + sy * sp * sr
	q[0] = cy * cp * sr - sy * sp * cr
	q[1] = sy * cp * sr + cy * sp * cr
	q[2] = sy * cp * cr - cy * sp * sr

	return q

	
# rotate a vector by a quaternion
def qv_mult(q1, v1):
	q2 = np.append(v1, 0.0)
	return tfs.quaternion_multiply(tfs.quaternion_multiply(q1, q2), tfs.quaternion_conjugate(q1))[:3]


'''
# transform axis angle representation into quaternian
def axang2quat(vector):
	l = np.linalg.norm(vector)
	s = math.sin(l / 2)
	x = vector[0] / l * s
	y = vector[1] / l * s
	z = vector[2] / l * s
	w = math.cos(l / 2)
	return np.array([x, y, z, w])
'''

# Convert rotation vector to quaternion	
def rvec2quat(vector):
	theta = np.linalg.norm(vector)
	unit_vector = vector/theta
	sin_th_by_2 = math.sin(theta/2)
	x = unit_vector[0] * sin_th_by_2
	y = unit_vector[1] * sin_th_by_2
	z = unit_vector[2] * sin_th_by_2
	w = math.cos(theta/2)
	return np.array([x, y, z, w])

# find an average of a list of waypoints in position and heading
def find_average(latest_gates):
	count = len(latest_gates)

	pos = np.array([0, 0, 0])
	sin = 0
	cos = 0
	for gate in latest_gates:
		pos = pos + gate.pos
		sin = sin + math.sin(gate.hdg)
		cos = cos + math.cos(gate.hdg)

	pos = pos/count
	angle = math.atan2(sin, cos)

	return WP(pos, angle)


# find standard deviation of position of waypoints
def find_std_dev_waypoints(average, wp_input_history):
	deviation = 0
	for gate in wp_input_history:
		deviation = deviation + (np.linalg.norm(average.pos - gate.pos))**2
	deviation = math.sqrt(deviation/len(wp_input_history))
	return deviation


# set a value to a minimum
def min_value(value, minimum):
	if -minimum < value < 0:
		return -minimum
	elif 0 < value < minimum:
		return minimum
	else:
		return value


# limit a value at a maximum
def limit_value(value, limit):
	if value > limit:
		return limit
	elif value < -limit:
		return -limit
	else:
		return value

def wrap2pi(theta):
	while theta > np.pi or theta <= -np.pi:
		if theta > np.pi:
			theta = -2 * np.pi + theta
		elif theta <= -np.pi:
			theta = 2 * np.pi + theta
	return theta

# Kalman Filter
class KalmanFilter: 
  def __init__(self, A, B, C, Q, R, X0, dt=1):
	self.A = A
	self.B = B
	self.C = C
	self.Q = Q
	self.R = R
	self.P = Q
	self.X = X0
	self.X_dot = np.zeros(X0.shape, float)
	self.dt = dt
	
  def predict(self, U):
	current_X_dot = ( np.dot(self.A, self.X) + np.dot(self.B, U) )
	self.X = self.X + (current_X_dot + self.X_dot) / 2 * self.dt
	self.X_dot = current_X_dot
	
	F = np.eye(self.A.shape[0]) + self.A
	self.P = np.dot(F, np.dot(self.P, F.T)) + self.Q
	
	
  def update(self, Y):
	S = np.float32 (np.dot(self.C, np.dot(self.P, self.C.T)) + self.R)
  
	if S.shape[0] > 1:
	  self.K = np.dot(self.P, np.dot(self.C.T, np.linalg.inv(S)))
	else:
	  self.K = np.dot(self.P, self.C.T / S)
	  
	self.X = self.X + np.dot(self.K, Y - np.dot(self.C, self.X))
	self.P =  np.dot(np.eye(self.P.shape[0]) - np.dot(self.K, self.C), self.P)
	  
  def filter(self, U, Y):
	self.predict(U)
	self.update(Y)
	
# Extended Kalman Filter
class ExtendedKalmanFilter(KalmanFilter):
  def __init__(self, predictFunc, jacobianA, jacobianB, C, Q, R, X0, dt=1):
	self.jacobianA = jacobianA
	self.jacobianB = jacobianB
	self.C = C
	self.Q = Q
	self.R = R
	self.P = Q
	self.X = X0
	self.dt = dt
	self.predictFunc = predictFunc
	
  def calculateJacobians(self, X,U):
	self.A = self.jacobianA(X,U)
	self.B = self.jacobianB(X,U)
	
  def predict(self, U): 
	self.calculateJacobians(self.X, U)
	self.X = self.predictFunc(self.X,U,self.dt)
	self.P = np.dot(self.A, np.dot(self.P, self.A.T)) + self.Q
   

# Moving average filter
class MVA:
	def __init__(self, window = 5):
		self.window = window
		self.val_history = []
		self.filtered = 0
		
	def update(self, val):
		if len(self.val_history) < self.window:
			self.val_history.append(val)
			self.filtered = np.sum(np.array(self.val_history), axis=0)/len(self.val_history)
		else:
			self.val_history.pop(0)
			self.val_history.append(val)
			self.filtered = np.sum(np.array(self.val_history), axis=0)/self.window
		return self.filtered

	
# Get standard deviation of time series values
class STDEV:
	def __init__(self, window):
		self.window = window
		self.val_history = []
		self.stdev = 1e6
	
	def get_stdev(self, val):
		if 5 < len(self.val_history) < self.window:
			self.val_history.append(val)
		elif len(self.val_history) >= self.window:
			self.val_history.pop(0)
			self.val_history.append(val)
		else:
			pass
				
		self.stdev = np.std(self.val_history, axis=0)
		return self.stdev

# waypoint class with position and hdg as well as a string function
class WP:
	def __init__(self, pos, hdg):
		self.pos = np.array(pos)
		self.hdg = hdg

	def __str__(self):
		return str(list(self.pos) + [self.hdg])


class Gate:
	def __init__(self,gate_format,center_pos,hdg):


		self.pos = WP(np.array(center_pos),None)
		self.org_pos = WP(self.pos.pos,None)
		self.hdg = hdg        
		self.format = gate_format
		self.current_gate = None

		self.look_pos = self.pos
		self.exit_pos = WP(self.pos.pos + 20.0 * np.array([math.cos(self.hdg),math.sin(self.hdg),0]),None)


	def reset(self):
		self.pos = self.org_pos
		self.current_gate = None
		self.look_pos = WP(self.org_pos.pos - 5.0 * np.array([math.cos(self.hdg),math.sin(self.hdg),0]),None)
		self.exit_pos = WP(self.org_pos.pos + 20.0 * np.array([math.cos(self.hdg),math.sin(self.hdg),0]),None)

		
	def update_format(self,gate_format):

		if self.format == 'vertical' and gate_format == 'top':
			self.current_gate = gate_format
			self.pos = self.org_pos + np.array([0,0,.7])
			self.exit_pos = WP(self.pos.pos + 20.0 * np.array([math.cos(self.hdg),math.sin(self.hdg),0]),None)
		

		elif self.format == 'vertical' and gate_format == 'bottom':
			self.current_gate = gate_format
			self.pos = self.org_pos - np.array([0,0,.7])
			self.exit_pos = WP(self.pos.pos + 20.0 * np.array([math.cos(self.hdg),math.sin(self.hdg),0]),None)


		elif self.format == 'horizontal' and gate_format == 'left':
			self.current_gate = gate_format
			self.pos = self.org_pos + np.array([0.7,0.0,0.0])
			self.exit_pos = WP(self.pos.pos + 20.0 * np.array([math.cos(self.hdg),math.sin(self.hdg),0]),None)


		elif self.format == 'horizontal' and gate_format == 'right':
			self.current_gate = gate_format
			self.pos = self.org_pos - np.array([0.7,0.0,0.0])
			self.exit_pos = WP(self.pos.pos + 20.0 * np.array([math.cos(self.hdg),math.sin(self.hdg),0]),None)
	
	


class Bebop_Model:
	def __init__(self,pos,hdg):
		self.pos = np.array(pos) # global position
		self.vel = np.array([0.0,0.0,0.0]) # global velocity
		self.att = np.array([0.0,0.0,0.0]) # roll pitch yaw, FRD
		self.cmd_att = np.array([0.0,0.0,0.0,0.0]) # roll pitch yaw, FRD
		self.body_vel = np.array([0.0,0.0,0.0])

		self.max_tilt = 40*math.pi/180
		self.roll_rate = 1.5*math.pi
		self.yaw_rate = .8*math.pi
		self.climb_rate = .3
		self.drag_term = 0.0

		self.accel_scale = 1.0
		self.vel_scale = 0.5

		self.pose = Pose()
		
	def propagate(self,t):
		roll_accel = limit_value(9.81*math.tan(self.att[0]),self.max_tilt)
		pitch_accel = limit_value(9.81*math.tan(-self.att[1]),self.max_tilt)
		yaw = self.att[2]

		accel = self.accel_scale * np.array([pitch_accel*math.cos(yaw)-roll_accel*math.sin(yaw), roll_accel*math.sin(yaw)+pitch_accel*math.sin(yaw), self.cmd_att[2]*self.climb_rate])
		accel = accel - self.drag_term * .5 * np.array([self.vel[0]**2, self.vel[1]**2, 0])
		self.vel = self.vel + accel * t
		self.pos = self.pos + self.vel_scale * self.vel*t
		
		
		
		# roll_err = self.cmd_att[0] - self.att[0]
		# pitch_err = self.cmd_att[1] - self.att[1]

		# if abs(roll_err) > self.roll_rate * t:
		# 	d_roll = np.sign(roll_err) * self.roll_rate * t
		# else:
		# 	d_roll = roll_err

		# if abs(pitch_err) > self.roll_rate * t:
		# 	d_pitch = np.sign(pitch_err) * self.roll_rate * t
		# else:
		# 	d_pitch = pitch_err
		
		# d_yaw = self.cmd_att[3] * self.yaw_rate * t
		

		# self.att = self.att + np.array([d_roll, d_pitch, d_yaw])
		
		self.update_pose()


	def update_pose(self):

		self.pose.position.x = self.pos[0]
		self.pose.position.y = self.pos[1]
		self.pose.position.z = self.pos[2]

		quat = tfs.quaternion_from_euler(self.att[0],self.att[1],self.att[2])
		self.pose.orientation.x = quat[0]
		self.pose.orientation.y = quat[1]
		self.pose.orientation.z = quat[2]
		self.pose.orientation.w = quat[3]


	# updates based solely on new odometry
	def update_odom(self,odom):
		pos_t = odom.pose.pose.position
		self.pos = np.array([pos_t.x, pos_t.y, pos_t.z])

		quat = odom.pose.pose.orientation
		self.att = np.array(tfs.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w]))
		
		vel_twist = odom.twist.twist.linear
		self.vel = np.array([vel_twist.x, vel_twist.y, vel_twist.z])
		
		self.pose = odom.pose.pose


	# updates commanded attitude from commands
	def update_att(self,commands):
		self.cmd_att[0] = self.max_tilt*commands[0]
		self.cmd_att[1] = -self.max_tilt*commands[1]
		self.cmd_att[2] = commands[2]
		self.cmd_att[3] = commands[3]
	

	# Update based on relative gate location
	def update_pose_gate(self,d_pos):
		meas_vec = np.array([data.pos.x, data.pos.y, data.pos.z])
		meas_hdg = data.hdg * 1.0

		# gate_hdg = gate.hdg
		gate_pos = gate.pos

		R_body2track = tfs.rotation_matrix(self.att[2],(0,0,1))[0:3,0:3]
		
		
		gate_proj = np.matmul(R_body2track, meas_vec)
		pos_new = gate_pos.pos - gate_proj
		
		self.vel = np.matmul(R_body2track,self.body_vel)
		

		self.pos = pos_new
		
		self.update_pose()


	def get_drone_pose(self):
		drone_pose = Drone_Pose()
		drone_pose.pos = self.pose
		drone_twist = Twist()
		drone_twist.linear.x = self.vel[0]
		drone_twist.linear.y = self.vel[1]
		drone_twist.linear.z = self.vel[2]
		drone_pose.vel = drone_twist
		drone_pose.hdg = self.att[2]
		return drone_pose


	def update_body_vel(self,vel):
		self.body_vel[0] = vel.x
		self.body_vel[1] = vel.y
		self.body_vel[2] = vel.z


	def update_orientation(self,odom):
		
		quat = odom.pose.pose.orientation
		att_temp = tfs.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
		self.att = np.array(att_temp)

		temp_vel = odom.twist.twist.linear
		odom_vel = np.array([temp_vel.x, temp_vel.y, temp_vel.z])
		self.body_vel = odom_vel

		R_body2track = tfs.rotation_matrix(self.att[2],(0,0,1))[0:3,0:3]
		self.vel = np.matmul(R_body2track,odom_vel)





# PID control loop without smoothing
class PID:
	def __init__(self, p=2.0, i=0.0, d=1.0, derivator=0, integrator=0, integrator_max=0.5,
				integrator_min=-0.5):
		self.kp = p
		self.ki = i
		self.kd = d
		self.derivator = derivator
		self.integrator = integrator
		self.integrator_max = integrator_max
		self.integrator_min = integrator_min
		self.error = 0.0

		self.p_value = None
		self.i_value = None
		self.d_value = None


	def reset(self):
		self.integrator = 0
		self.derivator = None

	def update(self, err):
		if self.derivator is None:
			self.derivator = err

		self.error = err

		self.p_value = self.kp * self.error
		self.d_value = self.kd * (self.error - self.derivator)
		self.derivator = self.error

		self.integrator = self.integrator + self.error

		if self.integrator > self.integrator_max:
			self.integrator = self.integrator_max
		elif self.integrator < self.integrator_min:
			self.integrator = self.integrator_min

		self.i_value = self.integrator * self.ki

		return [self.p_value, self.i_value, self.d_value]


# PID control loop with some derivator smoothing
class PID2:
	def __init__(self, p=2.0, i=0.0, d=1.0, derivator=[0.0, 0.0, 0.0, 0.0], integrator=0, integrator_max=.5, integrator_min=-.5):
		self.kp = p
		self.ki = i
		self.kd = d
		self.derivator = derivator
		self.integrator = integrator
		self.integrator_max = integrator_max
		self.integrator_min = integrator_min

		self.p_value = None
		self.i_value = None
		self.d_value = 0.0    

	def update(self, err):
		if self.derivator is None:
			self.derivator = [err, err, err, err]

		self.d_value = self.kd * ((4 * err - sum(self.derivator)) / 10)

		self.derivator.pop(0)
		self.derivator.append(err)      

		self.p_value = self.kp * err
		self.integrator = self.integrator + err

		if self.integrator > self.integrator_max:
			self.integrator = self.integrator_max
		elif self.integrator < self.integrator_min:
			self.integrator = self.integrator_min

		self.i_value = self.integrator * self.ki

		return [self.p_value, self.i_value, self.d_value]

	def reset(self):
		self.integrator = 0
		self.derivator = None


# bebop class definition
class Bebop:
	# BEBOP STATE overview
	#   0   landed
	#   1   takeoff
	#   2   hovering
	#   3   flying
	#   4   landing
	#   5   emergency
	#   6   not observed (usertakeoff, User take off state. Waiting for user action to take off)
	#   7   not observed (for fixed wing, motor ramping state)
	#   8   not observed (emergency landing after sensor defect. Only YAW is taken into account)

	def __init__(self):
		pass
	LANDED = 0
	TAKEOFF = 1
	HOVERING = 2
	FLYING = 3
	LANDING = 4
	EMERGENCY = 5
	USERTAKEOFF = 6
	MOTOR_RAMP = 7
	SENSOR_DEFECT = 8

