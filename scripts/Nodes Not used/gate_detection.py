#!/usr/bin/python
'''
Program description: The gate_detector takes in the left camera image, identifies
                    the 4 corners of the prominant gate and calculates the gate pose 
                    using P3P algorithm.
Author: Swapneel Naphade (snaphade@umd.edu)
version: 2.0
Date: 6/19/19
Change log: 
    7/5/19: Added the gate pose publisher
'''

import rospy
import signal
import sys
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Pose
import numpy as np
import cv2
from common_resources import MVA, quat2euler, euler2quat, qv_mult, rvec2quat, array2pose
from time import time
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

# HSV thresholds for gate
hsv_thresh_low = (0, 0, 230)
hsv_thresh_high = (180, 150, 255)

# MVA filters
orientationFilter = MVA(10)
translationFilter = MVA(10)

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

def rectifyStereoImage(img):
    # Camera Matrix 720p
    cameraMatrix = np.array([[700, 0.0, 640], [0.0, 700, 360], [0.0, 0.0, 1.0]])

    # Distortion Coefficients 720p
    distCoeffs = np.array([-0.1740500032901764, 0.028304599225521088, 0.0, 0.0, 0.0])

    height, width, depth = img.shape
    half_width = int(width/2)

    start = time()
    # Separate left and right images
    left_img = img[:height, :half_width]
    right_img = img[:height, half_width:width]

    # Rectify images
    left_img = cv2.undistort(left_img, cameraMatrix, distCoeffs)
    right_img = cv2.undistort(right_img, cameraMatrix, distCoeffs)
    undistorted_img = np.concatenate((left_img, right_img), axis=1)

    return undistorted_img

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

def image_raw_callback(ros_image):
    global bridge, hsv_thresh_high, hsv_thresh_low, orientationFilter, translationFilter
    img = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    start = time()
    gate = detect_gate(img, hsv_thresh_low, hsv_thresh_high)
    #rospy.loginfo("Gate detection time: {} s.\n".format(time() - start))	
     
    if gate is not None and np.array_equal(gate.shape, [4,2]): 
        cv2.drawContours(img, [gate.reshape(4,2)], 0, (255, 0, 0), 2)	
        annotateCorners(gate, img)
        euler, tvec = getGatePose(gate, 1.15) # Gate size in meters
        
        euler = orientationFilter.update(euler)
        tvec = translationFilter.update(tvec)
        gate_quat = euler2quat(euler)
        
        # gate_pose = Pose()
        # gate_pose.position.x = tvec[0]
        # gate_pose.position.y = tvec[1]
        # gate_pose.position.z = tvec[2]
        # gate_pose.orientation.w = gate_quat[3]
        # gate_pose.orientation.x = gate_quat[0]
        # gate_pose.orientation.y = gate_quat[1]
        # gate_pose.orientation.z = gate_quat[2]
        
        gate_pose = array2pose(tvec, gate_quat)
        
        gate_pose_pub.publish(gate_pose)
    else:
        gate_pose_pub.publish(Pose())
        # gate_detection.publish(Image())
        
    #img = cv2.resize(img, (0,0), fx=0.5, fy=0.5)
    gate_detection.publish(bridge.cv2_to_imgmsg(img, "bgr8"))

if __name__ == '__main__':

    signal.signal(signal.SIGINT, signal_handler)

    # Set simulation parameter to True. The system will start in simulation mode by default.
    SIMULATION = rospy.get_param("/simulation")
            
    # Initialize node
    rospy.init_node('gate_detector', anonymous=False)

    # Subcribers
    # state subscription
    # rospy.Subscriber('/state', String, state_callback)

    # Camera image subscription
    if SIMULATION:
        rospy.Subscriber('/ardrone/stereo/rgb/image_raw', Image, image_raw_callback)
    else:
        rospy.Subscriber('/autodrone/image_raw', Image, image_raw_callback)

    # Publishers
    # Image publication
    gate_detection = rospy.Publisher('/gate_detection_image', Image, queue_size=2)

    # Gate pose publisher
    gate_pose_pub = rospy.Publisher('/gate_pose', Pose, queue_size=5)

    # Update rate for the control loop
    rate = rospy.Rate(100) # 100 hz

    while not rospy.is_shutdown():
        rate.sleep()
