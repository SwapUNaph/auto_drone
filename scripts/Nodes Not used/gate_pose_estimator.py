#!/usr/bin/python
'''
Program description: This script estimates the position and orientation of
					the gate using extended kalman filter.
Author: Swapneel Naphade (snaphade@umd.edu)
version: 1.0
Date: 9/3/19
Change log: 

'''

import rospy
import signal
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, Quaternion, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Empty, Bool, Header
from auto_drone.msg import Traj_error
from common_resources import *
import numpy as np

drone_vel = Twist()

def signal_handler(_, __):
    # enable Ctrl+C of the script
    sys.exit(0)
    
def drone_odometry_callback(drone_odom):
	global drone_vel
	drone_vel = drone_odom.twist.twist	

def gatePoseDynamics(X,U,dt=1,noise=False):
  v = U[:3].reshape(1,3)
  w = U[3:].reshape(1,3)
  x = X[:3].reshape(1,3)
  theta = X[3:].reshape(1,3)
  x = x + ( -v + np.cross(w,x) ) * dt
  theta = theta - w * dt 
  X = np.append(x,theta,axis=0).reshape(6,1) 
  if noise:
    X = X + np.random.randn(X.shape[0], 1)*0.1
  return X


def jacobianA(X,U):
  return np.array([[0,-U[5],U[4],0,0,0],
          [U[5],0,-U[3],0,0,0],
          [-U[4],U[3],0,0,0,0],
          [0,0,0,0,0,0],
          [0,0,0,0,0,0],
          [0,0,0,0,0,0]])

def jacobianB(X,U):
  return np.array([[-1,0,0,0,X[2],-X[1]],
                    [0,-1,0,X[2],0,X[0]],
                    [0,0,-1,X[1],-X[0],0],
                    [0,0,0,-1,0,0],
                    [0,0,0,0,-1,0],
                    [0,0,0,0,0,-1]] )
 
# C,Q,R
C = np.eye(6, dtype='float')      
Q = np.zeros((6, 6), float)
np.fill_diagonal(Q, 0.01)

R = np.zeros((C.shape[0], C.shape[0]), float)
np.fill_diagonal(R, 0.01)

# Initial Estimate
X0 = np.zeros((6,1))

# Instantiate EKF
EKF = ExtendedKalmanFilter(gatePoseDynamics, jacobianA, jacobianB, C, Q, R, X0, dt=1/60)	

def gate_pose_callback(gate_pose_wrt_drone):
	global drone_pose, EKF
	
	drone_line_vel, drone_ang_vel = twist2array(drone_vel)
	gate_position_wrt_drone, gate_pose_wrt_drone_ornt_arr = pose2array(gate_pose_wrt_drone)
	
	if not np.array_equal(gate_pose_wrt_drone_ornt_arr , np.zeros(4)) and not (gate_pose_wrt_drone is None):
		Y = np.append(gate_position_wrt_drone, quat2euler(gate_pose_wrt_drone_ornt_arr)).reshape((6,1))
		U = np.append(drone_line_vel, drone_ang_vel).reshape((6,1))
		EKF.filter(U,Y)
		
		X = EKF.X
		filtered_gate_pose = array2pose(X[:3], euler2quat(X[3:]))
		filtered_gate_pose_pub.publish(filtered_gate_pose)
	else:
		filtered_gate_pose_pub.publish(gate_pose_wrt_drone)

	
if __name__ == '__main__':
	
	signal.signal(signal.SIGINT, signal_handler)
	
	
	# Set simulation parameter to True. The system will start in simulation mode by default.
	SIMULATION = rospy.get_param("/simulation")

	
	# Initialize node
	rospy.init_node('gate_pose_estimator', anonymous=False)
	
	# Subcribers	
	# Gate Pose wrt drone
	rospy.Subscriber('/gate_pose', Pose, gate_pose_callback)
	
	# Drone odometry subscription
	if SIMULATION:
		rospy.Subscriber('/ground_truth/state', Odometry, drone_odometry_callback)
		takeoff_publisher = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)	
		land_publisher = rospy.Publisher('/ardrone/land', Empty, queue_size=1)	
	else:
		rospy.Subscriber('/bebop/odom', Odometry, drone_odometry_callback)
		takeoff_publisher = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)	
		land_publisher = rospy.Publisher('/bebop/land', Empty, queue_size=1)
		
	# Corrected drone pose subscription
	#rospy.Subscriber('/drone_pose', Pose, drone_pose_callback)
	
	
	# Publishers
	# filtered_gate_pose publication
	filtered_gate_pose_pub = rospy.Publisher('/filtered_gate_pose', Pose, queue_size=1)

	
	# Update rate for the estimator loop
	rate = rospy.Rate(60) # 50 hz
	
	while not rospy.is_shutdown():		
		rate.sleep()
	
	
