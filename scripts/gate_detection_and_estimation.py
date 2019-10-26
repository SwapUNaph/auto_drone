#!/usr/bin/python
'''
Program description: The node gets the drone pose, velocity from bebop node
                        and publishes the gate pose (raw and filtered) and gate
                        type (left, right, up or down).
Author: Swapneel Naphade (snaphade@umd.edu)
version: 1.0
Date: 10/4/19
Change log: 
    
'''

import rospy
import signal
import sys
import math
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Pose
from auto_drone.msg import Drone_Pose, WP_Msg, Detection_Active
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged
from nav_msgs.msg import Odometry
import numpy as np
import cv2
from common_resources import *
from time import time
from cv_bridge import CvBridge, CvBridgeError
from tf import transformations as tfs


######################  Detection Parameters  ##########################

# Gate size in meters
GATE_SIZE = 1.43
NOT_GATE_SIZE = 1.18


# HSV thresholds for LED gate
hsv_thresh_low = (0, 0, 250)
hsv_thresh_high = (180, 50, 255)

# HSV thresholds for non-LED gate
not_gate_hsv_thresh_low = (0, 0, 0)
not_gate_hsv_thresh_high = (180, 255, 100)

# Gate thresholds
AREA_THRESH = 1000
ASPECT_RATIO_THRESH_LOW = 0.7 # Should be between 0.0 and 1.0
ASPECT_RATIO_THRESH_HIGH = 1/ASPECT_RATIO_THRESH_LOW
SOLIDITY_THRESH = 0.90
ROI_MEAN_THRESH = 100

DETECTION_ACTIVE = True
GATE_TYPE_VERTICAL = True

########################################################################

########################### Global Variables  ##########################

# Camera Capture
cap = cv2.VideoCapture(0)
cap.set(3, 2560) # Width 2560x720
cap.set(4, 720) # Height
cap.set(5, 60) # FPS
FRAME_WIDTH = cap.get(3)

bridge = CvBridge()

drone_position = np.zeros(3, float)
drone_orientation = np.zeros(3, float)
drone_linear_vel = np.zeros(3, float)
drone_angular_vel = np.zeros(3, float)

BATTERY_THRESH = 20 # Battery threshold for stopping

########################################################################

##########################  Filters   ##################################
# MVA filters
orientationFilter = MVA(20)
translationFilter = MVA(5)
headingBiasFilter= MVA(20)
noGateTvecFilter = MVA(20)

# Loop Frequency (important for kalman filtering)
LOOP_FREQ = 60

#Process Noise and Sensor noise
PROCESS_NOISE = 0.01	# Variance of process noise
SENSOR_NOISE = 0.1		# Variance of sensor noise

                    
# Headingcomplementory filter gain
HEADING_FILTER_GAIN = 0.3  # Weight for measurement reading

'''
def gatePoseDynamics(X,U,dt=1,noise=False):
  v = U[:3].reshape(1,3)
  w = U[3:].reshape(1,3)
  x = X[:3].reshape(1,3)
  theta = X[3:].reshape(1,1)
  x = x + ( -v + np.cross(w,x) ) * dt
  theta = theta - w * dt 
  X = np.append(x,theta,axis=0).reshape(6,1) 
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
 
'''

# A,B,C,Q,R
A = np.zeros((3, 3), float)
B = -np.eye(3, dtype='float')
C = np.eye(3, dtype='float')      
Q = np.zeros((3, 3), float)
np.fill_diagonal(Q, PROCESS_NOISE)


R = np.zeros((C.shape[0], C.shape[0]), float)
np.fill_diagonal(R, SENSOR_NOISE)

# Initial Estimate
X0 = np.zeros((3,1))

# Instantiate KF
#KF = ExtendedKalmanFilter(gatePoseDynamics, jacobianA, jacobianB, C, Q, R, X0, dt=1.0/LOOP_FREQ)
KF = KalmanFilter(A,B,C,Q,R,X0,dt=0.1)

# Heading estimator
def filterHeading(measuredHdg, droneHdg):
    
    headingBias = headingBiasFilter.update(measuredHdg + droneHdg)
    
    if np.cos(headingBias) >= 0.5:
        headingBias = 0
    elif np.cos(headingBias) <= -0.5:
        headingBias = np.pi
        
    return (measuredHdg + HEADING_FILTER_GAIN * (measuredHdg - (headingBias - droneHdg)) )

        
########################################################################
	
def signal_handler(_, __):
    # enable Ctrl+C of the script
    sys.exit(0)
    
def logMeasuredGatePose(X):
    euler = X[3:] * 180.0 / math.pi
    rospy.loginfo("\nPosition: {} (m) \nOrientation: {}(deg)\n".format(X[:3],euler))
    
    
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

def detect_gate(img, hsv_thresh_low, hsv_thresh_high, areaIndex):
    '''
    Description: The function takes in an image and hsv threshold values, detects 
                the largest 4-sided polygon and returns its corner coordinates.
    @params: 
    img: CV2 RGB Image
    hsv_threshold_low: Low threshold for color detection in HSV format, numpy array of length 3
    hsv_threshold_high: High threshold for color detection in HSV format, numpy array of length 3

    @return:
    contour: Numpy array of 4 coordinates of contour corners or None if no gate detected
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

    # Draw all contours
    #contour_img = img.copy()
    #cv2.drawContours(contour_img, contours, -1, (0, 255, 0), 1)
    #cv2.imshow('contour_img', contour_img)
    
    # Filter contour by area: area > 5 % of image area
    contours = list(filter(lambda x: (cv2.contourArea(x) > AREA_THRESH) , contours))
    #print("Contours after area filter: %d" % len(quadrlFiltered))
    
    # Approximate quadrilaterals
    quadrlFiltered=[]
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.05*cv2.arcLength(cnt,True),True)
        if len(approx) == 4:
            quadrlFiltered.append(approx)
            
    #print("Contours before all filters: %d" % len(quadrl))
            
    # Filter contour by area: area > 5 % of image area
    #quadrlFiltered = list(filter(lambda x: (cv2.contourArea(x) > AREA_THRESH) , quadrlFiltered))
    #print("Contours after area filter: %d" % len(quadrlFiltered))

    # Filter contour by solidity > 0.9
    quadrlFiltered = list(filter(lambda x: (solidity(x) > SOLIDITY_THRESH) , quadrlFiltered))
    #print("Contours after solidity filter: %d" % len(quadrlFiltered))

    # Filter contour by aspect ratio: 1.20 > AR > 0.8
    quadrlFiltered = list(filter(lambda x: (aspectRatio(x) > ASPECT_RATIO_THRESH_LOW) & (aspectRatio(x) < ASPECT_RATIO_THRESH_HIGH) , quadrlFiltered))
    #print("Contours after aspect ratio filter: %d" % len(quadrlFiltered))

    # Filter contour by ROI mean
    quadrlFiltered = list(filter(lambda x: contourROIMean(x, blur) < ROI_MEAN_THRESH , quadrlFiltered))
    #print("Contours after aspect ratio filter: %d" % len(quadrlFiltered))

    #print("Square contour areas:")
    #for sq in quadrlFiltered:
        #print(cv2.contourArea(sq))

    # Sort quadrilaterals by area
    quadrlFiltered = sorted(quadrlFiltered, key=lambda x: cv2.contourArea(x))

    
    if len(quadrlFiltered) > 0:
        # Get the largest contour
        gate_contour = quadrlFiltered[areaIndex].reshape(4,2)
        
        # Sort the points starting with top left and going counter-clockwise
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
    else:
        print("No gate detected!")
        return None

def getGatePose(contour, gate_side):
    '''
    Description: The function takes in gate contour points in counter clockwise direction starting from
                top left corner and dimensions of gate side and outputs the relative positin of the gate    
                wrt drone.
    @params: 
    contour: 4x2 numpy array of contour points
    gate_side: Gate side dimension (preferably in meters)

    @return:
    (tvec, euler): translation vector and euler angles (in radians) of the gate center 
                    wrt drone (NWU coordinate system).
    '''

    objectPoints = np.array([
        (-gate_side/2, -gate_side/2, 0.0),
        (-gate_side/2, gate_side/2, 0.0),
        (gate_side/2, gate_side/2, 0.0),
        (gate_side/2, -gate_side/2, 0.0)
    ])

    #Camera Transformation wrt drone
    dRc = np.array([[0,0,1],[-1,0,0],[0,-1,0]])

    ############################  For CaliCam    ##################################
    #cameraMatrix = np.array([[ 223.47508211, 0.0, 348.03017129], [0.0, 225.41305757, 215.59249353], [0.0, 0.0, 1.0]] )
    #distCoeffs = np.array([-0.26163237,  0.05491025,  0.0056113,  -0.0013107,  -0.00456478])

    ###############################################################################

    ####################################   For ZED stereo   #############################
    ## Camera Matrix 720p 
    cameraMatrix = np.array([[700, 0.0, 640], [0.0, 700, 360], [0.0, 0.0, 1.0]])

    ### Distortion Coefficients 720p
    distCoeffs = np.array([-0.1740500032901764, 0.028304599225521088, 0.0, 0.0, 0.0])

    # Camera Matrix VGA
    #cameraMatrix = np.array([[350, 0.0, 336], [0.0, 350, 188], [0.0, 0.0, 1.0]])

    # Distortion Coefficients VGA
    #distCoeffs = np.array([-0.17447000741958618, 0.027922799810767174, 0.0, 0.0, 0.0])

    #####################################################################################

    # Solve perspective 3 point problem
    (success, rvec, tvec) = cv2.solvePnP(objectPoints, contour.reshape(4,2).astype(float), cameraMatrix, distCoeffs, cv2.SOLVEPNP_P3P)
    rvec = np.squeeze(rvec)
    tvec = np.squeeze(tvec)

    # Transform tvec and euler angles to drone coordinates
    tvec = np.matmul(dRc, tvec)
    euler = np.matmul(dRc, rvec.T)

    return (euler, tvec) # euler angles in radians

def drone_odometry_callback(corrected_drone_odom):
    global drone_position, drone_orientation, drone_linear_vel, drone_angular_vel
    
    #drone_position, drone_orientation  =  pose2array(corrected_drone_odom.pos)
    #drone_orientation = quat2euler(drone_orientation)
    #drone_linear_vel, drone_angular_vel = twist2array(corrected_drone_odom.vel)
    
    drone_position, drone_orientation  =  pose2array(corrected_drone_odom.pose.pose)
    drone_orientation = quat2euler(drone_orientation)
    drone_linear_vel, drone_angular_vel = twist2array(corrected_drone_odom.twist.twist)

def battery_callback(battery_msg):
    if battery_msg.percent < BATTERY_THRESH:
        land_publisher.publish(Empty())
        rospy.logerr("Drone Battery < {} %.".format(BATTERY_THRESH))
        
def gate_detection_active_callback(det_active):
	global DETECTION_ACTIVE, GATE_TYPE_VERTICAL
	DETECTION_ACTIVE =  det_active.active
	GATE_TYPE_VERTICAL = det_active.vertical


def detect_gate_type(gate, not_gate, gate_detection_str):
	global GATE_TYPE_VERTICAL
	if gate_detection_str == "gate":
		difference = gate - not_gate
		if (np.linalg.norm(difference) - 1.4) <= 0.2: # Both gates are reliably detected
			if not GATE_TYPE_VERTICAL: # gate is horizontal
				if difference[1] > 0: # gate is left to no_gate
					gate_detection_str = "left"
				else:
					gate_detection_str = "right"
			else:
				if difference[2] > 0: # gate is vertical
					gate_detection_str = "up"
				else:
					gate_detection_str = "down"



if __name__ == '__main__':
	
	global DETECTION_ACTIVE, GATE_TYPE_VERTICAL

    signal.signal(signal.SIGINT, signal_handler)

    # Set simulation parameter to True. The system will start in simulation mode by default.
    #SIMULATION = rospy.get_param("/simulation")
            
    # Initialize node
    rospy.init_node('gate_detector', anonymous=False)

    # Subcribers
    # Detection state subscription
    #rospy.Subscriber('/state', Detection_Active, state_callback)
    
    # Drone odometry subscription
    #rospy.Subscriber('/auto/pose', Drone_Pose, drone_odometry_callback)
    rospy.Subscriber('/bebop/odom', Odometry, drone_odometry_callback)
    rospy.Subscriber('/bebop/states/common/CommonState/BatteryStateChanged', CommonCommonStateBatteryStateChanged, battery_callback)
    rospy.Subscriber('/auto/gate_detection_active', Detection_Active, gate_detection_active_callback)


    # Publishers
    image_publisher = rospy.Publisher('/auto/gate_detection_image', Image, queue_size=2)
    gate_pose_pub = rospy.Publisher('/auto/raw_gate_WP', WP_Msg, queue_size=5)
    filtered_gate_pose_pub = rospy.Publisher('/auto/filtered_gate_WP', WP_Msg, queue_size=5)
    land_publisher = rospy.Publisher('/bebop/land', Empty, queue_size=1)
    

    # Update rate for the control loop
    rate = rospy.Rate(LOOP_FREQ) # LOOP_FREQ hz

    gate_detection_string = "no_gate"
    start = time()
    
    while not rospy.is_shutdown():
        
        if DETECTION_ACTIVE:
			ret, img = cap.read()
			if img is None:
				rospy.logerr("No Camera detected!!!")
			else:
				# For Zed Stereo
				img = img[:,:int(FRAME_WIDTH/2)]
				
				# Gate detection           
				gate = detect_gate(img, hsv_thresh_low, hsv_thresh_high, -1)
				not_gate = detect_gate(img, not_gate_hsv_thresh_low, not_gate_hsv_thresh_high, 0)
				
				# If gate detected succesfully
				if gate is not None and np.array_equal(gate.shape, [4,2]):
					cv2.drawContours(img, [gate.reshape(4,2)], 0, (255, 0, 0), 2)
					annotateCorners(gate, img)  
					raw_euler, raw_tvec = getGatePose(gate, GATE_SIZE)
					raw_ng_euler, raw_ng_tvec = getGatePose(gate, GATE_SIZE)
					ng_tvec = noGateTvecFilter.update(raw_ng_tvec)
					
					if raw_euler[2] > np.pi or (np.linalg.norm(raw_tvec) > 30): # Outlier rejection
						gate_detection_string = "no_gate"
					else:
						gate_detection_string = "gate"                   
				else:
					gate_detection_string = "no_gate"
					
				if gate_detection_string == "no_gate":
					raw_euler = np.zeros(3, float)
					raw_tvec = np.zeros(3, float)
					KF.C = np.zeros(KF.C.shape, float) 	# No measurements, rely only on prediction
				else:  
					KF.C = np.eye(KF.C.shape[0], dtype='float') # Use measurements to fuse with predictions
					
				# Kalman Filtering
				euler = orientationFilter.update(raw_euler)         
				gateHeading = filterHeading(euler[2], drone_orientation[2])
		
				mva_tvec = translationFilter.update(raw_tvec)
				
				Y = mva_tvec.reshape((3,1))
				U = np.matmul( tfs.rotation_matrix(gateHeading, (0,0,1))[:3,:3], drone_linear_vel.reshape((3,1)) )
				KF.dt = (time() - start)
				start = time()
				KF.filter(U,Y)
				X = KF.X.copy()
				
				# Detect gate type
				gate_type = detect_gate_type(X, ng_tvec, gate_detection_string)
				
				# Send messages
				filtered_gate_WP = array2WP(X, gateHeading, gate_detection_string)
				gate_WP = array2WP(raw_tvec, raw_euler[2], "")
				gate_pose_pub.publish(gate_WP)
				filtered_gate_pose_pub.publish(filtered_gate_WP) 
				
			else:
				pass
            rospy.loginfo("LOOP Frequency: {} Hz".format(1/(time() - start)))
            #img = cv2.resize(img, None, fx=0.25, fy=0.25)   
            image_publisher.publish(bridge.cv2_to_imgmsg(img,"bgr8"))

            
