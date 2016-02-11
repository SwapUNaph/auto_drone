#!/usr/bin/env python
import rospy
import signal
import sys
import cv2
import math
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def callback_img(data):
	
	global bridge
	global pub_1
	global pub_2
	global pub_3

	img = bridge.imgmsg_to_cv2(data)
	print 'Callback: ',img.shape

	


	img_msg = bridge.cv2_to_imgmsg(img,encoding='mono8')
	pub_1.publish(img_msg)






def function(img,gate_pnts):
	scale = 1.4

	TL = np.squeeze(gate_pnts[0:1,:])
	BL = np.squeeze(gate_pnts[1:2,:])
	BR = np.squeeze(gate_pnts[2:3,:])
	TR = np.squeeze(gate_pnts[3:4,:])

	w = max(gate_pnts[:,0:1]) - min(gate_pnts[:,0:1])
	h = max(gate_pnts[:,1:2]) - min(gate_pnts[:,1:2])

	P_tl = TL + (TL - TR)
	P_bl = BL + (BL - BR)
	P_tr = TR + (TR - TL)
	P_br = BR + (BR - BL)

	circle_width = int(1.5 * (.2*(w+h)/2.0))
	cv2.circle(img,(P_tl[0], P_tl[1]),circle_width,(0,255,0))
	cv2.circle(img,(P_tr[0], P_tr[1]),circle_width,(0,255,0))
	cv2.circle(img,(P_bl[0], P_bl[1]),circle_width,(0,255,0))
	cv2.circle(img,(P_br[0], P_br[1]),circle_width,(0,255,0))

	cnt_x_L = (TL[0] + BL[0] + P_tl[0] + P_bl[0])/4.0
	cnt_y_L = (TL[1] + BL[1] + P_tl[1] + P_bl[1])/4.0

	cnt_x_R = (TR[0] + BR[0] + P_tr[0] + P_br[0])/4.0
	cnt_y_R = (TR[1] + BR[1] + P_tr[1] + P_br[1])/4.0

	# print cnt_x_L
	# print cnt_x_R
	# print cnt_y_L
	# print cnt_y_R

	dim_L = [int(cnt_x_L - .5 * w * scale),int(cnt_x_L + .5 * w * scale),int(cnt_y_L - .5 * h * scale),int(cnt_y_L + .5 * h * scale)]
	dim_R = [int(cnt_x_R - .5 * w * scale),int(cnt_x_R + .5 * w * scale),int(cnt_y_R - .5 * h * scale),int(cnt_y_R + .5 * h * scale)]
	
	imgr = img[dim_R[2]:dim_R[3], dim_R[0]:dim_R[1]]
	imgl = img[dim_L[2]:dim_L[3], dim_L[0]:dim_L[1]]

	# img_R = np.copy(imgr)
	# img_L = np.copy(imgl)

	imgr = cv2.GaussianBlur(imgr,(5,5), 5)
	imgl = cv2.GaussianBlur(imgl,(5,5), 5)
	


	imgr = cv2.erode(imgr, np.ones((3,3), dtype=np.uint8), iterations=1)
	imgl = cv2.erode(imgl, np.ones((3,3), dtype=np.uint8), iterations=1)


	# # Remove blobs
	# minSize = 10000
	# # get connectivity data
	# nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(img, connectivity=8)
	# nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(img, connectivity=8)

	# # remove background image
	# stats = stats[1:]; nb_components -= 1
	# # create return image
	# img = np.zeros((output.shape), dtype=np.uint8)
	# # for every component in the image, keep only if it's above minSize
	# for i in range(nb_components):
	#    if stats[i,4] >= minSize:
	# 	   img[output == i + 1] = 255



	# img = cv2.GaussianBlur(img,(3,3), 3)

	# Canny
	canny_minVal=50
	canny_maxVal=250
	canny_grad=3
	imgr = cv2.Canny(imgr, canny_minVal, canny_maxVal, None, canny_grad)
	imgl = cv2.Canny(imgl, canny_minVal, canny_maxVal, None, canny_grad)


	imgr = cv2.dilate(imgr, np.ones((3,3), dtype=np.uint8), iterations=1)
	imgl = cv2.dilate(imgl, np.ones((3,3), dtype=np.uint8), iterations=1)

	
	# Hough Lines
	hough_pixelRes=1
	hough_angleRes=np.pi/180.
	hough_minNumIntersections=150
	hough_minLineLength=50
	hough_maxLineGap=20
	hough_lineExtension=25.0



	linesR = cv2.HoughLinesP(imgr, hough_pixelRes, hough_angleRes, hough_minNumIntersections, None, hough_minLineLength, hough_maxLineGap)
	linesL = cv2.HoughLinesP(imgl, hough_pixelRes, hough_angleRes, hough_minNumIntersections, None, hough_minLineLength, hough_maxLineGap)
	
	theta_accept = 15

	theta_min = theta_accept * np.pi/180.
	theta_max = (90-theta_accept) * np.pi/180.


	# if linesR is not None:
	# 	for i in range(0, len(linesR)):
	# 		l = linesR[i][0]
	# 		theta = math.atan(abs(l[3]-float(l[1]))/abs(float(l[2])-l[0]))
	# 		if theta < theta_min or theta > theta_max:
	# 			cv2.line(img_R, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv2.LINE_AA)

	# if linesL is not None:
	# 	for i in range(0, len(linesL)):
	# 		l = linesL[i][0]
	# 		theta = math.atan(abs(l[3]-float(l[1]))/abs(float(l[2])-l[0]))
	# 		if theta < theta_min or theta > theta_max:
	# 			cv2.line(img_L, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv2.LINE_AA)


	P_tl = P_tl - [dim_L[0],dim_L[2]]
	P_bl = P_bl - [dim_L[0],dim_L[2]]
	# cv2.circle(img_L,(P_tl[0], P_tl[1]),20,(0,0,255))
	# cv2.circle(img_L,(P_bl[0], P_bl[1]),20,(0,0,255))
	
	P_tr = P_tr - [dim_R[0],dim_R[2]]
	P_br = P_br - [dim_R[0],dim_R[2]]
	# cv2.circle(img_R,(P_tr[0], P_tr[1]),20,(0,0,255))
	# cv2.circle(img_R,(P_br[0], P_br[1]),20,(0,0,255))


	squeez = np.squeeze(linesR)

	dx = 1.0 * np.absolute(squeez[:,3] - squeez[:,1])
	dy = 1.0 * np.absolute(squeez[:,2] - squeez[:,0])
	dydx = dx / dy

	theta_vec = np.arctan(dydx)

	ba1 = theta_vec < theta_min
	ba2 = theta_vec > theta_max
	squeez = squeez[ba1 | ba2,:]

	pnts = squeez[:,0:2]
	pnts = np.concatenate((pnts,squeez[:,2:4]), axis=0)
	
	# for a in pnts:
		# cv2.circle(img_R,(a[0], a[1]),2,(0,255,0))

	dtr = P_tr-pnts
	dbr = P_br-pnts

	dist_tr = np.linalg.norm(dtr,axis=1)
	dist_br = np.linalg.norm(dbr,axis=1)
		
	sum_tr = sum(np.maximum(2 * circle_width - dist_tr,0))
	sum_br = sum(np.maximum(2 * circle_width - dist_br,0))
	
	right_sum = sum_tr*sum_br

	# print 'Right_sum: ',right_sum


	squeez = np.squeeze(linesL)
	
	dx = 1.0 * np.absolute(squeez[:,3] - squeez[:,1])
	dy = 1.0 * np.absolute(squeez[:,2] - squeez[:,0])
	dydx = dx / dy

	theta_vec = np.arctan(dydx)

	ba1 = theta_vec < theta_min
	ba2 = theta_vec > theta_max
	squeez = squeez[ba1 | ba2,:]

	pnts = squeez[:,0:2]
	pnts = np.concatenate((pnts,squeez[:,2:4]), axis=0)
	
	# for a in pnts:
		# cv2.circle(img_L,(a[0], a[1]),2,(0,255,0))

	dtl = P_tl-pnts
	dbl = P_bl-pnts

	dist_tl = np.linalg.norm(dtl,axis=1)
	dist_bl = np.linalg.norm(dbl,axis=1)
		
	sum_tl = sum(np.maximum(2 * circle_width - dist_tl,0))
	sum_bl = sum(np.maximum(2 * circle_width - dist_bl,0))
	
	left_sum = sum_tl*sum_bl

	# print 'Left_sum: ',left_sum

	left_sum = left_sum / (left_sum+right_sum)
	right_sum = 1 - left_sum


	# return img_R, img_L
	return left_sum, right_sum







def hough_stuff(img):

	
	img_t = np.copy(img)

	# img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	gate2_low = np.array([20, 5, 5])
	gate2_high = np.array([50, 40, 40])

	hsv_low = np.array([0,0,80])
	hsv_high = np.array([180,255,255])

	# 0 0 247 180 20 255

	# return img
	img = cv2.GaussianBlur(img,(5,5), 5)

	img = cv2.inRange(img, gate2_low, gate2_high)
	# img = cv2.inRange(img, hsv_low, hsv_high)

	# return img



	# img =  cv2.erode(img, np.ones((3,3), dtype=np.uint8), iterations=1)
	


	# Remove blobs
	if bool(1):
		minSize = 10000
		# get connectivity data
		nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(img, connectivity=8)
		# remove background image
		stats = stats[1:]; nb_components -= 1
		# create return image
		img = np.zeros((output.shape), dtype=np.uint8)
		# for every component in the image, keep only if it's above minSize
		for i in range(nb_components):
		   if stats[i,4] >= minSize:
			   img[output == i + 1] = 255



	# img = cv2.GaussianBlur(img,(3,3), 3)

	# Canny
	canny_minVal=50
	canny_maxVal=250
	canny_grad=3
	img = cv2.Canny(img, canny_minVal, canny_maxVal, None, canny_grad)

	img = cv2.dilate(img, np.ones((3,3), dtype=np.uint8), iterations=1)

	
	# Hough Lines
	hough_pixelRes=1
	hough_angleRes=np.pi/180.
	hough_minNumIntersections=150
	hough_minLineLength=50
	hough_maxLineGap=20
	hough_lineExtension=25.0



	linesP = cv2.HoughLinesP(img, hough_pixelRes, hough_angleRes, hough_minNumIntersections, None, hough_minLineLength, hough_maxLineGap)
	
	theta_min = 15 * np.pi/180.
	theta_max = 75 * np.pi/180.


	if linesP is not None:
		for i in range(0, len(linesP)):
			l = linesP[i][0]
			
			
			theta = math.atan(abs(l[3]-float(l[1]))/abs(float(l[2])-l[0]))
			# print l,', ',theta
			if theta < theta_min or theta > theta_max:
				cv2.line(img_t, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv2.LINE_AA)

	tl = (460,90)
	tr = (650,85)
	bl = (455,275)
	br = (650,270)
	
	x = tl[0]
	y = tl[1]
	w = 650-455
	h = 275-85
	scale = 1.2

	# cv2.circle(img_t,bl,10,(0,255,0))

	proj_tl = (x-w, y)
	proj_bl = (x-w, y+h)

	proj_tr = (x+2*w, y)
	proj_br = (x+2*w, y+h)

	circle_width = int(1.5 * (.2*(w+h)/2.0))
	cv2.circle(img_t,proj_tl,circle_width,(0,255,0))
	cv2.circle(img_t,proj_bl,circle_width,(0,255,0))
	cv2.circle(img_t,proj_tr,circle_width,(0,255,0))
	cv2.circle(img_t,proj_br,circle_width,(0,255,0))







	p1 = np.array([tr[0], tr[1]])
	p2 = np.array([br[0], br[1]])

	squeez = np.squeeze(linesP)

	dx = 1.0 * np.absolute(squeez[:,3] - squeez[:,1])
	dy = 1.0 * np.absolute(squeez[:,2] - squeez[:,0])
	dydx = dx / dy

	# print dydx

	theta_vec = np.arctan(dydx)
	
	ba1 = theta_vec < theta_min
	ba2 = theta_vec > theta_max

	squeez = squeez[ba1 | ba2,:]


	pnts = squeez[:,0:2]
	pnts = np.concatenate((pnts,squeez[:,2:4]), axis=0)
	
	d1 = p1-pnts
	d2 = p2-pnts

	dist_1 = np.linalg.norm(d1,axis=1)
	dist_2 = np.linalg.norm(d2,axis=1)
		
	close_1 = 2 * circle_width - dist_1
	close_1 = np.maximum(close_1,0)
	sum_1 = sum(close_1)

	close_2 = 2 * circle_width - dist_2
	close_2 = np.maximum(close_2,0)
	sum_2 = sum(close_2)

	right_sum = sum_2*sum_1

	print 'Right_sum: ',right_sum


	p1 = np.array([tl[0], tl[1]])
	p2 = np.array([bl[0], bl[1]])

	squeez = np.squeeze(linesP)

	pnts = squeez[:,0:2]
	pnts = np.concatenate((pnts,squeez[:,2:4]), axis=0)
	
	d1 = p1-pnts
	d2 = p2-pnts

	dist_1 = np.linalg.norm(d1,axis=1)
	dist_2 = np.linalg.norm(d2,axis=1)
		
	close_1 = 2 * circle_width - dist_1
	close_1 = np.maximum(close_1,0)
	sum_1 = sum(close_1)

	close_2 = 2 * circle_width - dist_2
	close_2 = np.maximum(close_2,0)
	sum_2 = sum(close_2)

	left_sum = sum_2*sum_1

	print 'Left_sum: ',left_sum









	return img_t




def signal_handler(_, __):
	sys.exit(0)




if __name__ == '__main__':

	signal.signal(signal.SIGINT, signal_handler)
	rospy.init_node('image_processing', anonymous=False)


	pub_1 = rospy.Publisher('/visual/test_1', Image, queue_size=1)
	pub_2 = rospy.Publisher('/visual/test_2', Image, queue_size=1)
	pub_3 = rospy.Publisher('/visual/test_3', Image, queue_size=1)
	
	bridge = CvBridge()

	
	rospy.Subscriber('/visual/test_hough', Image, callback_img)
	
	# rospy.spin()

	img = cv2.imread('test_image.png')

	# img = hough_stuff(img)


	g_pnts = np.array([[460,90],[455,275],[650,270],[650,85]])

	left,right = function(img,g_pnts)

	print 'Left: ',left
	print 'Right: ',right
	# cv2.imshow('left',right)
	# cv2.imshow('right',left)

	key = cv2.waitKey(-1)

	cv2.destroyAllWindows()
