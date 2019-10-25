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





def hough_stuff(img):


	# img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	img_t = np.copy(img)

	hsv_gate2_low = np.array([20, 5, 5])
	hsv_gate2_high = np.array([50, 40, 40])
	# 0 0 247 180 20 255

	# return img
	img = cv2.GaussianBlur(img,(7,7), 5)

	img = cv2.inRange(img, hsv_gate2_low, hsv_gate2_high)





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
	hough_minNumIntersections=120
	hough_minLineLength=50
	hough_maxLineGap=20
	hough_lineExtension=25.0






	# lines = cv2.HoughLines(img, hough_pixelRes, hough_angleRes, hough_minNumIntersections, None, hough_minLineLength, hough_maxLineGap)

	# if lines is not None:
	# 	print 'len: ',len(lines)

	# 	for i in range(0, len(lines)):
	# 		rho = lines[i][0][0]
	# 		theta = lines[i][0][1]
	# 		a = math.cos(theta)
	# 		b = math.sin(theta)
	# 		x0 = a * rho
	# 		y0 = b * rho
	# 		pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
	# 		pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
	# 		cv2.line(img, pt1, pt2, (255,255,255), 5, -1)

	
	# img_n = np.zero(img.shape[0],img.shape[1])


	linesP = cv2.HoughLinesP(img, hough_pixelRes, hough_angleRes, hough_minNumIntersections, None, hough_minLineLength, hough_maxLineGap)
	
	theta_min = 20 * np.pi/180.
	theta_max = 70 * np.pi/180.



	if linesP is not None:
		for i in range(0, len(linesP)):
			l = linesP[i][0]
			
			
			theta = math.atan(abs(l[3]-float(l[1]))/abs(float(l[2])-l[0]))
			print l,', ',theta
			if theta < theta_min or theta > theta_max:
				cv2.line(img_t, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv2.LINE_AA)


	# # Hough intersections
	# if not(np.any(lines)):
	#    print "ERROR: no hough lines exist for getHoughIntersections."
	#    return None

	# # clear any existing hough intersections
	# points = []
	# check_list = np.array([[0,0]], dtype=np.int32)

	# # there are smarter ways to do this but eh
	# for line1 in self._hough_lines:
	#    for line2 in self._hough_lines:

	#         # check if the line normals are 'different enough'
	#         if not(line1.checkNormals(line2, tol=1.0)):

	#            # if they are, get the intesection point then double check
	#            # that you actually got a point back
	#            int_point = line1.getIntersectionPoint(line2)
	#            if int_point:

	#                # round point to the nearest int and ensure it's unique
	#                # before adding to the final list
	#                int_point.roundToInt()
	#                if not([int_point.x, int_point.y] in check_list.tolist()):
	#                    check_list = np.append(check_list, np.array([[int_point.x, int_point.y]], dtype=np.int32), axis=0)
	#                    points.append([int_point.x, int_point.y])

	# self._hough_intersectionPoints_p = np.array(points, dtype=np.float32)

	# if self.record_image:
	#    for p in self._hough_intersectionPoints_p:
	#        cv2.circle(self.dst_color, (int(p[0]), int(p[1])), 2, (0, 255, 0), -1)

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

	img = hough_stuff(img)
	print img.shape

	cv2.imshow('img',img)

	key = cv2.waitKey(-1)

	cv2.destroyAllWindows()