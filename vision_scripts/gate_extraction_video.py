#!/usr/bin/python
import cv2
import numpy as np
from time import time
from matplotlib.colors import hsv_to_rgb

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
	cv2.imshow('hsv', hsv2rgb(hsv))

	# Mask
	mask = cv2.inRange(hsv, hsv_thresh_low, hsv_thresh_high)
	cv2.imshow('mask', mask)

	# Find contours
	im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	#print("Contour_list: {}".format(contours))
	
	# Print solidity
	#print("Contour solidities:")
	#for cnt in contours:
		#print(solidity(cnt))
	
	# Draw all contours
	contour_img = img.copy()
	cv2.drawContours(contour_img, contours, -1, (0, 0, 255), 1)
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

	# Filter for contour solidity > 0.8
	quadrlFiltered = list(filter(lambda x: (solidity(x) > 0.80) , quadrlFiltered))
	print("Contours after solidity filter: %d" % len(quadrlFiltered))
	
	# Filter by contour aspect ratio: 1.30 > AR > 0.7
	quadrlFiltered = list(filter(lambda x: (aspectRatio(x) > 0.70) & (aspectRatio(x) < 1.30) , quadrlFiltered))
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
	#img = cv2.imread('images/gates.png', 1)	
	
	#cap = cv2.VideoCapture(0)
	#frame_width = int(cap.get(3))
	#frame_height = int(cap.get(4))

	cap = cv2.VideoCapture('videos/green_gate.MOV')
	frame_width = int(cap.get(3) / 3.0)
	frame_height = int(cap.get(4) / 3.0)
	print(frame_width, frame_height)
		 
	# Define the codec and create VideoWriter object.The output is stored in 'outpy.avi' file.
	out = cv2.VideoWriter('videos/green_gate_detected.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 30, (frame_width,frame_height))
		
	hsv_thresh_low = (30, 20, 150)
	hsv_thresh_high = (100, 255, 255)

	while(cap.isOpened()):
		ret, img = cap.read()		
		if ret:
			img = cv2.resize(img,(frame_width,frame_height))
			cnt = detect_gate(img, hsv_thresh_low, hsv_thresh_high)
			cv2.drawContours(img, [cnt], 0, (0,0,255), 3)
			out.write(img)  
			cv2.imshow('image',img)
			if cv2.waitKey(25) & 0xFF == ord('q'):
				break
		else:
			break

	cap.release()
	out.release()
	cv2.destroyAllWindows()




if __name__ == '__main__':
	main()
