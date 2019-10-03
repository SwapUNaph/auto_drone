#!/usr/bin/env python

# Script developed by Vincenz Frenzel (PIDs by Derek Thompson, frequency extraction by Matt Vaughn,
#					orientation conversions, Kalman Filter and MVA filter by Swapneel Naphade)
#  --- Changelog ---
# Goal:     Collect information that is needed at multiple locations of other scripts or that are less essential
# Status:   11/03/18: Started changelog and commented code
#			7/9/19: Added Moving average filter, orientation conversions

import math
from tf import transformations as tfs
from geometry_msgs.msg import Twist, Pose, Quaternion
import numpy as np

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

# how fast do scripts run (I think this is never used)
frequency = 5

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


# transform axis angle representation into quaternian
def axang2quat(vector):
    l = np.linalg.norm(vector)
    s = math.sin(l / 2)
    x = vector[0] / l * s
    y = vector[1] / l * s
    z = vector[2] / l * s
    w = math.cos(l / 2)
    return np.array([x, y, z, w])

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


# calculate rotation periods from angles and timestamps
def calculate_periods(input_data):
    angles = np.unwrap(input_data[1, :])
    times = input_data[0, :]
    d_a = np.diff(angles)
    d_t = np.diff(times)
    return 2*math.pi * d_t / d_a

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
    self.dt = dt
    
  def predict(self, U):   
    self.X = self.X + ( np.dot(self.A, self.X) + np.dot(self.B, U) ) * self.dt
    F = np.eye(self.A.shape[0]) + self.A
    self.P = np.dot(F, np.dot(self.P, F.T)) + self.Q
    
  def update(self, Y):
    V = Y - np.dot(self.C, self.X)
    S = np.dot(self.C, np.dot(self.P, self.C.T)) + self.R
    S = np.float32(S)
    if S.shape[0] > 1:
      self.K = np.dot(self.P, np.dot(self.C.T, np.linalg.inv(S)))
    else:
      self.K = np.dot(self.P, self.C.T / S)
      
    self.X = self.X + np.dot(self.K, V)
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
		self.filtered = []
		
	def update(self, val):
		if len(self.val_history) < self.window:
			self.val_history.append(val)
			return np.zeros(val.shape)
		else:
			if True : 	#np.linalg.norm(np.subtract(self.filtered, val)) < 10:  # Disturbance rejection (not yet implemented)
				self.val_history.pop(0)
				self.val_history.append(val)
				self.filtered = np.sum(np.array(self.val_history), axis=0)/self.window
				return self.filtered
			else:
				return self.filtered
	

# waypoint class with position and hdg as well as a string function
class WP:
    def __init__(self, pos, hdg):
        self.pos = np.array(pos)
        self.hdg = hdg

    def __str__(self):
        return str(list(self.pos) + [self.hdg])


# this is the data that is required to pass dynamic gate
class OpenloopData:
    def __init__(self):
        self.timer = None
        self.period = None
        self.theta = None
        self.time_taken_to_gate = 2.0
        self.std_dev = None

        self.rotate_perform = False
        self.counter = 0
        self.triggered = False


# PID control loop without smoothing
class PID:
    def __init__(self, p=2.0, i=0.0, d=1.0, derivator=0, integrator=0, integrator_max=0.5,
				integrator_min=-0.5, output_limit = (-1,1), deadband = (-0.001,0.001)):
        self.kp = p
        self.ki = i
        self.kd = d
        self.derivator = derivator
        self.integrator = integrator
        self.integrator_max = integrator_max
        self.integrator_min = integrator_min
        self.error = 0.0
        self.output_limit = output_limit
        self.deadband = deadband

        self.p_value = None
        self.i_value = None
        self.d_value = None

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
		output = self.p_value + self.i_value + self.d_value

		if self.deadband[0] <= output <= self.deadband[1]:
			output = 0
		else: 
			pass
				
		if output <= self.output_limit[0]:
			output = self.output_limit[0]
		else:
			pass
				
		if output >= self.output_limit[1]:
			output = self.output_limit[1]
		else:
			pass

				
		return output

    def reset(self):
        self.integrator = 0
        self.derivator = None


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


# everything after here is used for determining rotation speed of dynamic gate
def fourier(data, freq, plot_freq_domain, plot_imag):
    # Convert data to numpy arrays
    global cur_ax, ax, nplots
    dnp = np.array(data)
    fnp = np.array(freq)
    # Tally # of data points
    L = max(np.shape(dnp))
    # Make sure frequency is correct shape
    if (fnp.ndim > 1 and np.shape(fnp)[1] > 1):
        fnp = np.transpose(fnp)

    # respones(freq) = 1/N SUM{k = 1 to N: data(t_k) * exp(-2*pi*j*freq*t_k)}
    # where frequency is modulated to find peak response for freq.
    # Create exp part of transform:
    exp_mat = dnp[:, 0]
    exp_mat = (-2 * math.pi * 1j) * exp_mat
    exp_mat = np.expand_dims(exp_mat, 1)
    fnp = np.expand_dims(fnp, 1)
    exp_mat = fnp * np.transpose(exp_mat)
    exp_mat = np.exp(exp_mat)

    # Create data part of fourier transform:
    source = np.cos(dnp[:, 1])
    source = np.expand_dims(source, 1)

    # Multiply the two together to get whole fourier transform
    val = np.matmul(exp_mat, source) / L

    # # Plotting
    # freq_dom = None
    # if (plot_freq_domain):
    #     cur_ax += 1
    #     ax = plt.subplot(nplots, 1, cur_ax)
    #     freq_dom = plt.plot(freq, np.real(val))
    # return val, freq_dom
    return val


# def fourier_imag_plane(data, freq):
#     # All this is to plot fourier response in imaginary plane
#     global cur_ax, ax, nplots
#     dnp = np.array(data)
#     fnp = np.array(freq)
#     L = max(np.shape(dnp))
#     if (fnp.ndim > 1 and np.shape(fnp)[1] > 1):
#         fnp = np.transpose(fnp)
#
#     print(fnp)
#     exp_mat = dnp[:, 0]
#     exp_mat = (-2 * math.pi * 1j) * exp_mat
#     exp_mat = np.expand_dims(exp_mat, 1)
#     fnp = np.expand_dims(fnp, 1)
#     exp_mat = fnp * np.transpose(exp_mat)
#     exp_mat = np.exp(exp_mat)
#     source = np.cos(dnp[:, 1])
#     source = np.expand_dims(source, 1)
#     val = np.transpose(exp_mat) * source
#     valtot = np.matmul(exp_mat, source) / L
#     cur_ax += 1
#     ax = plt.subplot(nplots, 1, cur_ax, projection='polar')
#     imag_plane = plt.polar(np.angle(val), np.linalg.norm(val, 2, 1))
#     center = plt.polar(np.angle(valtot), np.linalg.norm(valtot, 2, 1), 'ro')
#     return imag_plane


# Find angle t seconds after last data point
def angle_in_t_seconds(data, freq, offset, t, plot_history, plot_propagation):
    # global cur_ax, ax, nplots
    dnp = np.array(data)
    t_augmented = dnp[-1, 0] + t  # time of interest with respect to original time frame
    angle = np.mod(t_augmented * 2 * math.pi * freq + (offset - math.pi / 2) * 2 * math.pi, 2 * math.pi)
    # print 'in ', t, ' seconds, angle is ', angle, '[rad]'

    # Rest is plotting
    # t_start = dnp[-1, 0]
    # t_end = dnp[-1, 0]
    # if (plot_history):
    #     t_start = dnp[0, 0]
    # if (plot_propagation):
    #     t_end = dnp[-1, 0] + t
    # print t_start, t_end, t
    # t_range = np.linspace(t_start, t_end, ((t_end - t_start) + t) * 10)
    # time_prop = None
    # if (plot_history or plot_propagation):
    #     cur_ax += 1
    #     ax = plt.subplot(nplots, 1, cur_ax)
    #     if (plot_history):
    #         original = plt.plot(dnp[:, 0], dnp[:, 1])
    #     time_prop = plt.plot(t_range,
    #                          np.mod(t_range * 2 * math.pi * freq + (offset - math.pi / 2) * 2 * math.pi, 2 * math.pi))
    # return angle, time_prop
    return angle


def extract_freq(data, resolution, plot_freq_domain, plot_imag):
    freq_range = np.arange(0, 2 * math.pi, resolution)
    # [response, four_plot] = fourier(data, freq_range, plot_freq_domain, plot_imag)
    response = fourier(data, freq_range, plot_freq_domain, plot_imag)
    real_r = np.real(response)  # Real part of response
    imag_r = np.imag(response)  # Imaginary part of response
    index = np.argmax(np.abs(real_r))  # index for largest real part of response
    freq = freq_range[index]  # Frequency for largest real response
    offset = imag_r[index]  # Offset for largest real response (phase shift to calculate future angle)

    # Plotting
    # if (plot_imag):
    #     imag_plot = fourier_imag_plane(data, freq)
    # # print 'arg: ', np.angle(response[index])
    # print 'freq: ', freq * 2 * math.pi, ', offset: ', np.mod((offset - math.pi / 2) * 2 * math.pi, 2 * math.pi), '[rad]'
    # return [freq, offset, four_plot, imag_plot]
    return [freq, offset]
