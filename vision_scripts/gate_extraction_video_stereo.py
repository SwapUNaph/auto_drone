#!/usr/bin/python
import cv2
#import tf.transformations as tfs
import numpy as np
import math
from time import strftime, localtime, time
import matplotlib.pyplot as plt

'''
VideoCapture settings:
0. CV_CAP_PROP_POS_MSEC Current position of the video file in milliseconds.
1. CV_CAP_PROP_POS_FRAMES 0-based index of the frame to be decoded/captured next.
2. CV_CAP_PROP_POS_AVI_RATIO Relative position of the video file
3. CV_CAP_PROP_FRAME_WIDTH Width of the frames in the video stream.
4. CV_CAP_PROP_FRAME_HEIGHT Height of the frames in the video stream.
5. CV_CAP_PROP_FPS Frame rate.
6. CV_CAP_PROP_FOURCC 4-character code of codec.
7. CV_CAP_PROP_FRAME_COUNT Number of frames in the video file.
8. CV_CAP_PROP_FORMAT Format of the Mat objects returned by retrieve() .
9. CV_CAP_PROP_MODE Backend-specific value indicating the current capture mode.
10. CV_CAP_PROP_BRIGHTNESS Brightness of the image (only for cameras).
11. CV_CAP_PROP_CONTRAST Contrast of the image (only for cameras).
12. CV_CAP_PROP_SATURATION Saturation of the image (only for cameras).
13. CV_CAP_PROP_HUE Hue of the image (only for cameras).
14. CV_CAP_PROP_GAIN Gain of the image (only for cameras).
15. CV_CAP_PROP_EXPOSURE Exposure (only for cameras).
16. CV_CAP_PROP_CONVERT_RGB Boolean flags indicating whether images should be converted to RGB.
17. CV_CAP_PROP_WHITE_BALANCE Currently unsupported
18. CV_CAP_PROP_RECTIFICATION Rectification flag for stereo cameras (note: only supported by DC1394 v 2.x backend currently)

'''

# Camera Matrix
cameraMatrix = np.array([[700.0499877929688, 0.0, 637.5999755859375],
							[0.0, 700.0499877929688, 382.5060119628906],
							[0.0, 0.0, 1.0]])
	
# Distortion Coefficients
distCoeffs = np.array([-0.1740500032901764, 0.028304599225521088, 0.0, 0.0, 0.0])
	
	

def rvec2quat(vector):
	l = np.linalg.norm(vector)
	sin_th = math.sin(l/2)
	x = vector[0] /l * sin_th
	y = vector[1] /l * sin_th
	z = vector[2] /l * sin_th
	w = math.cos(l/2)
	return np.array([x, y, z, w])


def quat2euler(q):
	 #q = [x,y,z,w]
	roll = np.arctan2( 2*(q[3]*q[0] + q[1]*q[2]), 1 - 2*(q[0]**2 + q[1]**2) )
	pitch = np.arcsin( 2*(q[3]*q[1] - q[0]*q[2]) )
	yaw = np.arctan2( 2*(q[3]*q[2] + q[0]*q[1]), 1 - 2*(q[1]**2 + q[2]**2) )
	return np.array([roll,pitch,yaw])


def solidity(contour):
	hull_area = cv2.contourArea(cv2.convexHull(contour))
	if (hull_area != 0) :
		return float(cv2.contourArea(contour))/hull_area
	else:
		return 0


def aspectRatio(contour):
	x,y,w,h = cv2.boundingRect(contour)
	return float(w)/h
	
	
def annotateCorners(contour, img):
	contour = contour.reshape(4,2).tolist()
	count = 1
	for point in contour:
		cv2.circle(img, (point[0], point[1]), 10, (255,255,0), 0)
		cv2.putText(img, str(count), (point[0], point[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)
		count = count + 1
	 
	 
def detect_gate_pose(img, hsv_thresh_low, hsv_thresh_high):
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
	height, width, depth = img.shape
	half_width = int(width/2)
	
	# Separate left and right images
	left_img = img[:height, :int(width/2)]
	right_img = img[:height, int(width/2):width]
	
	## Undistort images
	left_img = cv2.undistort(left_img, cameraMatrix, distCoeffs)
	right_img = cv2.undistort(right_img, cameraMatrix, distCoeffs)
	img = np.concatenate((left_img, right_img), axis=1)
	
	# Convert to HSV
	#hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	#cv2.imshow('hsv', hsv)
	
	
	# Convert to gray
	#gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	#cv2.imshow('hsv', hsv)
	
	## Mask
	#mask = cv2.inRange(hsv, hsv_thresh_low, hsv_thresh_high)
	##print(mask)
	#cv2.imshow('mask', mask)
	
	# Blur 
	#blur = cv2.GaussianBlur(mask,(3,3), 3)
	#cv2.imshow('Blur', blur)
	
	#cv2.imshow('undistort', undistort_img)
	
	# HSV, Mask and blur left image
	left_hsv = cv2.cvtColor(left_img, cv2.COLOR_BGR2HSV)
	#right_hsv = cv2.cvtColor(right_img, cv2.COLOR_BGR2HSV)
	left_mask = cv2.inRange(left_hsv, hsv_thresh_low, hsv_thresh_high)
	#right_mask = cv2.inRange(right_hsv, hsv_thresh_low, hsv_thresh_high)
	left_mask = cv2.GaussianBlur(left_mask,(3,3), 2)
	#right_mask = cv2.inRange(right_img, hsv_thresh_low, hsv_thresh_high)

	# gray image
	#left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
	#right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
	#gray_img = np.concatenate((left_gray, right_gray), axis=1)
	#cv2.imshow("Gray image", gray_img)
	
	## Binary image
	#ret,left_thresh = cv2.threshold(left_gray,127,255,cv2.THRESH_BINARY)
	#cv2.imshow("Left_thresh", left_thresh)
	
	#print(mask)feture 
	
	# SIFT feature detection
	#sift = cv2.SIFT()
	#kp = sift.detect(gray_img,None)
	#sift_img=cv2.drawKeypoints(gray_img,kp)
	#cv2.imshow('sift_img', sift_img)
	
	# Find contours
	contours, hierarchy = cv2.findContours(left_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	#print("Contour_list: {}".format(contours))
	
	# Print solidity
	#print("Contour solidities:")
	#for cnt in contours:
		#print(solidity(cnt))
	
	# Draw all contours
	#contour_img = left_img.copy()
	#cv2.drawContours(contour_img, contours, -1, (0, 255, 0), 1)
	#cv2.imshow('contour_img', contour_img)

	# Approximate quadrilaterals
	quadrl=[]
	for cnt in contours:
		approx = cv2.approxPolyDP(cnt,0.10*cv2.arcLength(cnt,True),True)
		if len(approx) == 4:
			quadrl.append(approx)
			
	#print("Contours before all filters: %d" % len(quadrl))
			
	# Filter contour by area: area > 500
	quadrlFiltered = list(filter(lambda x: (cv2.contourArea(x) > 1000) , quadrl))
	#print("Contours after area filter: %d" % len(quadrlFiltered))

	# Filter for contour solidity > 0.9
	#quadrlFiltered = list(filter(lambda x: (solidity(x) > 0.90) , quadrlFiltered))
	#print("Contours after solidity filter: %d" % len(quadrlFiltered))
	
	# Filter by contour aspect ratio: 1.20 > AR > 0.8
	quadrlFiltered = list(filter(lambda x: (aspectRatio(x) > 0.8) & (aspectRatio(x) < 1.35) , quadrlFiltered))
	#print("Contours after aspect ratio filter: %d" % len(quadrlFiltered))

	
	#floodfill_img = mask.copy()
	
	
	#print("Square contour areas:")
	#for sq in quadrlFiltered:
		#print(cv2.contourArea(sq))
	
	# Sort quadrilaterals by area
	quadrlFiltered = sorted(quadrlFiltered, key=lambda x: cv2.contourArea(x))
	
	
	if len(quadrlFiltered) > 0:
		gate_contour = quadrlFiltered[-1].reshape(4,2) # Return the largest square contour by area (returns coordinates of corners)
		
		# Sort the points starting with top left and going anti-clockwise
		center = gate_contour.mean(axis=0)
		gate_cnt_sorted = [[0,0]]*4
		for point in gate_contour:
			if point[0] < center[0] and point[1] < center[1]:
					gate_cnt_sorted[0] = point
			elif point[0] <= center[0] and point[1] >= center[1]:
					gate_cnt_sorted[1] = point
			elif point[0] > center[0] and point[1] < center[1]:
				gate_cnt_sorted[3] = point
			else:
				gate_cnt_sorted[2] = point
				
		gate_cnt_sorted = np.array(gate_cnt_sorted)
		
		convolve_img = []
		for left_point in gate_cnt_sorted:
			#left_point = gate_cnt_sorted[2]
			left_point.reshape(1,2)
			
			#print("H:{}, W:{}".format(height, width))
			
			# Extract Kernel from left gray image
			delta = int(0.02*half_width)
			kernel_left = left_img[left_point[1]-delta:left_point[1]+delta , left_point[0]-delta:left_point[0]+delta]
			#print("Left Point: {}".format(left_point))
			#print("delta: {}".format(delta))
			#print("kernel size: {}".format(kernel_left.shape))
			#cv2.imshow("kernel_left", kernel_left)

			# Get minimum cost point for the right image
			min_cost_x = delta
			min_cost = 1000000
			cost_function = []
			for x in range(delta+1, half_width-delta-1):
				kernel_right = right_img[left_point[1]-delta:left_point[1]+delta , x-delta:x+delta]
				if kernel_left.shape != kernel_right.shape:
					continue
				cost = np.square(np.subtract(kernel_left, kernel_right)).sum() + (left_point[0] - x)**2
				cost_function.append(cost)	
				if cost <= min_cost:
					min_cost_x = x
					min_cost = cost
				
			con_img = cv2.filter2D(right_img,-1,kernel_left)
			#convolve_img.append(con_img)
			
				
			print("Left x: {}".format(left_point[0]))
			print("Min cost x: {}".format(min_cost_x))

			#matched_right = right_img[left_point[1]-delta:left_point[1]+delta , min_cost_x-delta:min_cost_x+delta]
			#cv2.imshow("Matched", matched_right)
			
			# Draw matched portions
			cv2.rectangle(img, (left_point[0]-delta, left_point[1]-delta), (left_point[0]+delta, left_point[1]+delta), (255,0,0), 2)
			cv2.rectangle(img, (min_cost_x-delta+half_width, left_point[1]-delta), (min_cost_x+delta+half_width, left_point[1]+delta), (255,0,0), 2)
			
		
		cv2.imshow("Feature Image", img)
		

		cv2.imshow("Convolution image", con_img)
		
		# Calculate disparity between the two points
		disparity = left_point[0] - min_cost_x
		print("Disparity: {}".format(disparity))
		return disparity	
	else:
		print("No gate detected!")
		return np.nan


def rectifyStereoImage(img):
	height, width = img.shape[:2]
	
	left_img = img[:height, :int(width/2)]
	right_img = img[:height, int(width/2):width]
	
	
def getGatePosePnP(contour, objectPoints):
	## Camera Position wrt drone
	#dTc = [0, 0, 0]
	
	##Camera Rotation wrt drone
	dRc = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
	#dQuatC = tfs.quaternion_from_matrix(dRc)
	dQuatC = [ 0.5, -0.5, 0.5, -0.5 ]
	
	# Camera Matrix
	cameraMatrix = np.array([[700.0499877929688, 0.0, 637.5999755859375],
							[0.0, 700.0499877929688, 382.5060119628906],
							[0.0, 0.0, 1.0]])
	
	# Distortion Coefficients
	distCoeffs = np.array([-0.1740500032901764, 0.028304599225521088, 0.0, 0.0, 0.0])
	
	# Solve perspective n point algorithm
	(success, rvec, tvec) = cv2.solvePnP(objectPoints, contour.reshape(4,2).astype(float), cameraMatrix, distCoeffs)
	
	rvec = np.squeeze(rvec)
	tvec = np.squeeze(tvec)
	quat = rvec2quat(rvec)
	
	quat = tfs.quaternion_multiply(quat, dQuatC)
	tvec = np.matmul(dRc, tvec.reshape(3,1))
	
	
	#print("Q : {}\nT: {}".format(quat, tvec))
	
	return (quat, tvec)


def getDisparity(left_img, right_img, left_point):
	left_point.reshape(1,2)
	height, width, depth = left_img.shape
	left_img = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
	right_img = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
	#print("H:{}, W:{}".format(height, width))
	
	# Extract Kernel from left image
	delta = 20
	kernel_left = left_img[left_point[1]-delta:left_point[1]+delta , left_point[0]-delta:left_point[0]+delta]

	# Get minimum cost point for the right image
	min_cost_x = delta
	min_cost = 1000000
	for x in range(delta+1, width-delta-1, 5):
		kernel_right = right_img[left_point[1]-delta:left_point[1]+delta , x-delta:x+delta]
		cost = np.absolute(kernel_left - kernel_right).sum() + abs(left_point[0] - x)	
		if cost <= min_cost:
			min_cost_x = x
			min_cost = cost
	
	#print("Min cost x: {}".format(min_cost_x))
	#matched_right = right_img[left_point[1]-delta:left_point[1]+delta , min_cost_x-delta:min_cost_x+delta]
	#cv2.imshow("Matched", matched_right)
	
	# Calculate disparity between the two points
	disparity = left_point[0] - min_cost_x
	print("Disparity: {}".format(disparity))
	return disparity
	
	
def main():
	# Load image
	#img = cv2.imread('images/gates.png', 1)	
	
	#cap = cv2.VideoCapture(0)
	#cap.set(5, 100)
	
	# Gate points:
	#[[[299 118]]

	 #[[289 204]]

	 #[[377 215]]

	 #[[387 130]]]

	gate_side = 103
	objectPoints = np.array([
			(-gate_side/2, -gate_side/2, 0.0),
			(-gate_side/2, gate_side/2, 0.0),
			(gate_side/2, gate_side/2, 0.0),
			(gate_side/2, -gate_side/2, 0.0)
		])
	
	
	cap = cv2.VideoCapture("videos/dual_image_video_blue.mp4")
	#cap = cv2.VideoCapture(0)
	
	# 720p : 2560x720 : 60 FPS
	# WVGA : 1344x376 : 100 FPS
	cap.set(5, 100)	# FPS
	cap.set(3, 2560)# Image Width
	cap.set(4, 720)	# Image Height
	cap.set(10, 0.8) # Brightness
	cap.set(11, 0.5) # Contrast
	cap.set(12, 0.5) # Saturation
	#cap.set(13, 0.0) # Hue
	
	frame_width = int(cap.get(3) / 2)
	frame_height = int(cap.get(4))
	print(frame_width, frame_height)
	
	VIDEO_RECORDING = False
		 
	if VIDEO_RECORDING:
		# Define the codec and create VideoWriter object.The output is stored in 'outpy.avi' file.
		filename = 'videos/gate_detection_'+ strftime("%d-%b-%Y_%H-%M-%S", localtime()) + '.mp4'
		out = cv2.VideoWriter(filename,cv2.VideoWriter_fourcc('M','J','P','G'), 20, (frame_width,frame_height))
	
	## Color thresholds
	# Blue gate	
	hsv_thresh_low = (90, 50, 50)
	hsv_thresh_high = (130, 255, 255)
	
	# Green gate
	#hsv_thresh_low = (40, 50, 50)
	#hsv_thresh_high = (110, 255, 255)
	
	# Green gate rgb
	#hsv_thresh_low = (100, 200, 100)
	#hsv_thresh_high = (255, 255, 255)
	
	# Red gate
	#hsv_thresh_low = (130, 40, 30)
	#hsv_thresh_high = (200, 255, 255)
	
	# Orange gate
	#hsv_thresh_low = (0, 100, 100)
	#hsv_thresh_high = (20, 255, 255)


	while(cap.isOpened()):
		start = time()
		ret, img = cap.read()	
			
		if ret:
			#left_img = img[0:frame_height, 0:frame_width]
			#right_img = img[0:frame_height, frame_width: 2*frame_width]			
			
			pose = detect_gate_pose(img, hsv_thresh_low, hsv_thresh_high)
			
			#left_mask = cv2.inRange(cv2.cvtColor(left_img, cv2.COLOR_BGR2HSV), hsv_thresh_low, hsv_thresh_high)
			#right_mask = cv2.inRange(cv2.cvtColor(right_img, cv2.COLOR_BGR2HSV), hsv_thresh_low, hsv_thresh_high)
			#cv2.imshow('left_mask',left_mask)
			#cv2.imshow('right_mask',right_mask)
			
			# PnP algorithm
			#quat, tvec = getGatePose(left_cnt, objectPoints)
			#euler = quat2euler(quat)	
			
			#cv2.drawContours(left_img, [left_cnt.reshape(4,2)], 0, (0,0,255), 2)
			#cv2.circle(img, (center[0], center[1]), 10, (255,255,0), 0)
			#cv2.putText(img, "X: {}, Y: {}, Z: {}".format(tvec[0], tvec[1], tvec[2]), (10, frame_height - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)
			#cv2.putText(img, "Roll: {}, Pitch: {}, Yaw: {}".format(euler[0], euler[1], euler[2]), (10, frame_height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)
			
										
			#cv2.drawContours(img, [left_cnt], 0, (0,0,255), 2)
			##cv2.circle(img, (center[0], center[1]), 10, (255,255,0), 0)
			##cv2.putText(img, "X: {}, Y: {}, Z: {}".format(tvec[0], tvec[1], tvec[2]), (10, frame_height - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)
			##cv2.putText(img, "Roll: {}, Pitch: {}, Yaw: {}".format(euler[0], euler[1], euler[2]), (10, frame_height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)
			#annotateCorners(left_cnt, img)
			
			
			# Print statements
			print("Detection rate: {} Hz".format(1/float(time() - start)))				
			#print("Brightness: {} ".format(cap.get(10)))				
			#print("Saturation: {} ".format(cap.get(11)))				
			#print("Contrast: {} ".format(cap.get(12)))				
			#print("Hue: {} ".format(cap.get(13)))		
			
			if cv2.waitKey(50) & 0xFF == ord('q'):
				break
			if VIDEO_RECORDING:
				out.write(img)  
		
		else:
			break
	
	cap.release()
	if VIDEO_RECORDING:
		out.release()
	cv2.destroyAllWindows()




if __name__ == '__main__':
	main()
