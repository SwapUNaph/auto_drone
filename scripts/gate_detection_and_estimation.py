#!/usr/bin/python
'''
Program description: The node gets the drone pose, velocity and gate id 
                        and publishes the gate pose (raw and filtered), 
                        correct drone_pose.
Author: Swapneel Naphade (snaphade@umd.edu)
version: 1.0
Date: 10/4/19
Change log: 
    
'''

import rospy
import signal
import sys
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Pose
from auto_drone.msg import Drone_Pose, WP_Msg, Detection_Active
import numpy as np
import cv2
from common_resources import *
from time import time
from cv_bridge import CvBridge, CvBridgeError


######################  Detection Parameters  ##########################

# Gate size in m
GATE_SIZE = 1.15

# HSV thresholds for gate
hsv_thresh_low = (0, 0, 230)
hsv_thresh_high = (180, 150, 255)

DETECTION_ACTIVE = True
GATE_TYPE_VERTICAL = None
########################################################################

########################### Global Variables  ##########################
# Camera Capture
cap = cv2.VideoCapture(0)
cap.set(3, 2560) # Width 2560x720
cap.set(4, 720) # Height
cap.set(5, 60) # FPS
FRAME_WIDTH = cap.get(3)

bridge = CvBridge()

drone_position = np.zeros(3)
drone_orientation = np.zeros(3)
drone_linear_vel = np.zeros(3)
drone_angular_vel = np.zeros(3)
########################################################################

##########################  Filters   ##################################
# MVA filters
orientationFilter = MVA(5)
translationFilter = MVA(5)

# Loop Frequency (important for kalman filtering)
LOOP_FREQ = 60

#Process Noise and Sensor noise
PROCESS_NOISE = 0.01	# Variance of process noise
SENSOR_NOISE = 0.01		# Variance of sensor noise

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
np.fill_diagonal(Q, PROCESS_NOISE)

R = np.zeros((C.shape[0], C.shape[0]), float)
np.fill_diagonal(R, SENSOR_NOISE)

# Initial Estimate
X0 = np.zeros((6,1))

# Instantiate EKF
EKF = ExtendedKalmanFilter(gatePoseDynamics, jacobianA, jacobianB, C, Q, R, X0, dt=1.0/LOOP_FREQ)

########################################################################
	
def signal_handler(_, __):
    # enable Ctrl+C of the script
    sys.exit(0)
    
def aspectRatio(contour):
    x,y,w,h = cv2.boundingRect(contour)
    return float(w)/h

def solidity(contour):
    hull_area = cv2.contourArea(cv2.convexHull(contour))
    if (hull_area != 0) :
        return float(cv2.contourArea(contour))/hull_area
    else:
        return 0

def annotateCorners(contour, img):
    contour = contour.reshape(4,2).tolist()
    count = 1
    for point in contour:
        cv2.circle(img, (point[0], point[1]), 10, (0,255,255), 2)
        #cv2.putText(img, str(count), (point[0], point[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)
        count = count + 1

def contourROIMean(contour, img):
    x,y,w,h = cv2.boundingRect(contour)
    return img[y:y+h, x:x+w].mean()

def detect_gate(img, hsv_thresh_low, hsv_thresh_high):
    '''
    Description: The functiont takes in an image and hsv threshold values, detects 
                the largest 4-sided polygon and returns its corner coordinates.
    @params: 
    img: CV2 RGB Image
    hsv_threshold_low: Low threshold for color detection in HSV format, numpy array of length 3
    hsv_threshold_high: High threshold for color detection in HSV format, numpy array of length 3

    @return:
    contour: Numpy array of 4 coordinates of contour corners
    '''
    # Convert to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #cv2.imshow('hsv', hsv)

    # Mask
    mask = cv2.inRange(hsv, hsv_thresh_low, hsv_thresh_high)
    #print(mask)
    #cv2.imshow('mask', mask)

    # Blur 
    blur = cv2.GaussianBlur(mask,(3,3), 3)
    #cv2.imshow('Blur', blur)

    # Find contours
    im2, contours, hierarchy = cv2.findContours(blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #print("Contour_list: {}".format(contours))

    # Print solidity
    #print("Contour solidities:")
    #for cnt in contours:
        #print(solidity(cnt))

    # Draw all contours
    #contour_img = img.copy()
    #cv2.drawContours(contour_img, contours, -1, (0, 255, 0), 1)
    #cv2.imshow('contour_img', contour_img)

    # Approximate quadrilaterals
    quadrl=[]
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.05*cv2.arcLength(cnt,True),True)
        if len(approx) == 4:
            quadrl.append(approx)
            
    #print("Contours before all filters: %d" % len(quadrl))
            
    # Filter contour by area: area > 5 % of image area
    quadrlFiltered = list(filter(lambda x: (cv2.contourArea(x) > 500) , quadrl))
    #print("Contours after area filter: %d" % len(quadrlFiltered))

    # Filter for contour solidity > 0.9
    quadrlFiltered = list(filter(lambda x: (solidity(x) > 0.50) , quadrlFiltered))
    #print("Contours after solidity filter: %d" % len(quadrlFiltered))

    # Filter by contour aspect ratio: 1.20 > AR > 0.8
    quadrlFiltered = list(filter(lambda x: (aspectRatio(x) > 0.5) & (aspectRatio(x) < 2.0) , quadrlFiltered))
    #print("Contours after aspect ratio filter: %d" % len(quadrlFiltered))

    # Filter by contour mean
    quadrlFiltered = list(filter(lambda x: contourROIMean(x, blur) < 150 , quadrlFiltered))
    #print("Contours after aspect ratio filter: %d" % len(quadrlFiltered))

    #print("Square contour areas:")
    #for sq in quadrlFiltered:
        #print(cv2.contourArea(sq))

    # Sort quadrilaterals by area
    quadrlFiltered = sorted(quadrlFiltered, key=lambda x: cv2.contourArea(x))

    if len(quadrlFiltered) > 0:
        gate_contour = quadrlFiltered[-1].reshape(4,2)
        
        # Sort the points starting with top left and going anti-clockwise
        center = gate_contour.mean(axis=0)
        gate_cnt_sorted = [None]*4
        for point in gate_contour:
            if point[0] < center[0] and point[1] < center[1]:
                    gate_cnt_sorted[0] = point
            elif point[0] < center[0] and point[1] >= center[1]:
                    gate_cnt_sorted[1] = point
            elif point[0] >= center[0] and point[1] < center[1]:
                gate_cnt_sorted[3] = point
            else:
                gate_cnt_sorted[2] = point
                
        gate_cnt_sorted_np = np.array(gate_cnt_sorted)
        return gate_cnt_sorted_np
        #if :
            ##print("gate contour: {}".format(gate_cnt_sorted))
            #return gate_cnt_sorted_np # Return the largest square contour by area (returns coordinates of corners)
        #else:
            #print("No gate detected!")
            #return None
    else:
        print("No gate detected!")
        return None

def getGatePose(contour, gate_side):

    #gate_side = 1.15
    objectPoints = np.array([
            (-gate_side/2, -gate_side/2, 0.0),
            (-gate_side/2, gate_side/2, 0.0),
            (gate_side/2, gate_side/2, 0.0),
            (gate_side/2, -gate_side/2, 0.0)
        ])

    ##Camera Transformation wrt drone
    dRc = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    #dTc = np.array([[0,0,1,0],[-1,0,0, 0],[0,-1,0, 0], [0,0,0,1]])
    #dQuatC = tfs.quaternion_from_matrix(dTc)
    #dQuatC = [ 0.5, -0.5, 0.5, -0.5 ]

    ###################################  Camera matrix and distortion coefficients for CaliCam    ##################################
    # cameraMatrix = np.array([[ 258.58131479, 0.0 , 348.1852167 ], [0.0 , 257.25344992, 219.07752178], [0.0 , 0.0, 1.0]])
    # distCoeffs = np.array([[-0.36310169,  0.10981468,  0.0057042,  -0.001884,   -0.01328491]])

    ################################################################################################################################

    ####################################   For ZED stereo   #########################################################
    ## Camera Matrix 720p 
    cameraMatrix = np.array([[700, 0.0, 640], [0.0, 700, 360], [0.0, 0.0, 1.0]])

    ### Distortion Coefficients 720p
    distCoeffs = np.array([-0.1740500032901764, 0.028304599225521088, 0.0, 0.0, 0.0])

    # Camera Matrix VGA
    #cameraMatrix = np.array([[350, 0.0, 336], [0.0, 350, 188], [0.0, 0.0, 1.0]])

    # Distortion Coefficients VGA
    #distCoeffs = np.array([-0.17447000741958618, 0.027922799810767174, 0.0, 0.0, 0.0])

    #################################################################################################################

    # Solve perspective 3 point algorithm
    (success, rvec, tvec) = cv2.solvePnP(objectPoints, contour.reshape(4,2).astype(float), cameraMatrix, distCoeffs, cv2.SOLVEPNP_P3P)
    rvec = np.squeeze(rvec)
    tvec = np.squeeze(tvec)	
    quat = rvec2quat(rvec)

    # Transform tvec and euler angles to drone coordinates
    tvec = np.matmul(dRc, tvec)	
    euler = np.matmul(dRc, quat2euler(quat).T)

    #quat = tfs.quaternion_multiply(dQuatC, quat)
    #tvec = np.matmul(dRc, tvec.reshape(3,1))

    #print("Position(m):\t{}\nAngle(deg):\t{}\n-----------------------".format(tvec, euler))

    return (euler, tvec)

def drone_odometry_callback(corrected_drone_pose):
	global drone_position, drone_orientation, drone_linear_vel, drone_angular_vel
	drone_position, drone_orientation  =  pose2array(corrected_drone_pose.pos)
	drone_orientation = quat2euler(drone_orientation)
	drone_linear_vel, drone_angular_vel = twist2array(corrected_drone_pose.vel)

if __name__ == '__main__':

    signal.signal(signal.SIGINT, signal_handler)

    # Set simulation parameter to True. The system will start in simulation mode by default.
    #SIMULATION = rospy.get_param("/simulation")
            
    # Initialize node
    rospy.init_node('gate_detector', anonymous=False)

    # Subcribers
    # Detection state subscription
    #rospy.Subscriber('/state', Detection_Active, state_callback)
    
    # Drone odometry subscription
    rospy.Subscriber('/auto/pose', Drone_Pose, drone_odometry_callback)


    # Publishers
    # Image publication
    image_publisher = rospy.Publisher('/auto/gate_detection_image', Image, queue_size=2)
    gate_pose_pub = rospy.Publisher('/auto/raw_gate_WP', WP_Msg, queue_size=5)
    filtered_gate_pose_pub = rospy.Publisher('/auto/filtered_gate_WP', WP_Msg, queue_size=5)
    

    # Update rate for the control loop
    rate = rospy.Rate(LOOP_FREQ) # LOOP_FREQ hz

    while not rospy.is_shutdown():
        ret, img = cap.read()
        if img is None:
            rospy.logerr("No Camera detected!!!")
        else:
            img = img[:,:int(FRAME_WIDTH/2)]
            
            # Gate detection
            gate = detect_gate(img, hsv_thresh_low, hsv_thresh_high)
            # If gate detected succesfully
            if gate is not None and np.array_equal(gate.shape, [4,2]):
                cv2.drawContours(img, [gate.reshape(4,2)], 0, (255, 0, 0), 2)
                annotateCorners(gate, img)  
                euler, tvec = getGatePose(gate, GATE_SIZE)
                euler = orientationFilter.update(euler)
                tvec = translationFilter.update(tvec)
                gate_WP = array2WP(tvec, euler[2], "")
                EKF.C = np.eye(6) # Use measurements to fuse with predictions
            else:
				euler = np.zeros(3)
				tvec = np.zeros(3)
				gate_WP = WP_Msg()
				EKF.C = np.zeros((6,6), float) # No measurements, rely only on prediction

                
			# Kalman Filtering
            Y = np.append(tvec, euler).reshape((6,1))
            U = np.append(drone_linear_vel, drone_angular_vel).reshape((6,1))
            EKF.filter(U,Y)
            X = EKF.X
            filtered_gate_WP = array2WP(X[:3], X[5], "")

            gate_pose_pub.publish(gate_WP)
            filtered_gate_pose_pub.publish(filtered_gate_WP)    
            image_publisher.publish(bridge.cv2_to_imgmsg(img,"bgr8"))
         
        rate.sleep()
            
