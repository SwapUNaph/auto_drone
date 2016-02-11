import cv2
import numpy as np
from matplotlib.colors import hsv_to_rgb
from time import time

def solidity(contour):
	hull_area = cv2.contourArea(cv2.convexHull(contour))
	if (hull_area != 0) :
		return float(cv2.contourArea(contour))/hull_area
	else:
		return 0
	
def aspectRatio(contour):
	x,y,w,h = cv2.boundingRect(contour)
	return float(w)/h
	
def hsv2rgb(hsv):
	rgb = hsv.copy()
	for ind, px in enumerate(hsv):
		rgb[ind] = hsv_to_rgb(px)
	return rgb
	
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
<<<<<<< HEAD
	cv2.imshow('hsv', hsv)
=======
	cv2.imshow('hsv', hsv2rgb(hsv))
>>>>>>> 2311185ac19b3a0d9d61f516a6cbbde763faf9bd

	# Mask
	mask = cv2.inRange(hsv, hsv_thresh_low, hsv_thresh_high)
	cv2.imshow('mask', mask)

	# Find contours
	contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	#print("Contour_list: {}".format(contours))
	
	# Print solidity
	#print("Contour solidities:")
	#for cnt in contours:
		#print(solidity(cnt))
	
	# Draw all contours
	contour_img = img.copy()
	cv2.drawContours(contour_img, contours, -1, (0, 255, 0), 1)
	cv2.imshow('contour_img', contour_img)

	# Approximate quadrilaterals
	quadrl=[]
	for cnt in contours:
		approx = cv2.approxPolyDP(cnt,0.1*cv2.arcLength(cnt,True),True)
		if len(approx) == 4:
			quadrl.append(approx)
			
	print("Contours before all filters: %d" % len(quadrl))
			
	# Filter contour by area: area > 1000
	quadrlFiltered = list(filter(lambda x: (cv2.contourArea(x) > 1000) , quadrl))
	print("Contours after area filter: %d" % len(quadrlFiltered))

	# Filter for contour solidity > 0.9
	quadrlFiltered = list(filter(lambda x: (solidity(x) > 0.90) , quadrlFiltered))
	print("Contours after solidity filter: %d" % len(quadrlFiltered))
	
	# Filter by contour aspect ratio: 1 > AR > 0.9 
	quadrlFiltered = list(filter(lambda x: (aspectRatio(x) > 0.90) , quadrlFiltered))
	print("Contours after aspect ratio filter: %d" % len(quadrlFiltered))

	#print("Square contour areas:")
	#for sq in quadrlFiltered:
		#print(cv2.contourArea(sq))
	
	# Sort quadrilaterals by area
	quadrlFiltered = sorted(quadrlFiltered, key=lambda x: cv2.contourArea(x))
	
	
	if len(quadrlFiltered) > 0:
		return quadrlFiltered[-1]		# Return the largest square contour by area (returns coordinates of corners)
	else:
		print("No gate detected!")
		return np.array([[0, 0],[0, 0],[0, 0],[0, 0]])


def main():
	# Load image
<<<<<<< HEAD
	img = cv2.imread('images/gate2.png', 1)	
=======
	img = cv2.imread('images/gate3.png', 1)	
>>>>>>> 2311185ac19b3a0d9d61f516a6cbbde763faf9bd
	
	color_thresh_low = (0, 200, 50)
	color_thresh_high = (20, 255, 255)

	now = time()
	cnt = detect_gate(img, color_thresh_low, color_thresh_high)
	print("Detection time: {} s.".format( time() - now))
	print("Contour: {}".format( cnt ))

	cv2.drawContours(img, [cnt], 0, (255,0,0), 2)  
	cv2.imshow('image',img)
	k = cv2.waitKey(0)
	if k == 27:         # wait for ESC key to exit
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
