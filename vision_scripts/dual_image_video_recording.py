#!/usr/bin/python

import cv2
import numpy as np
from time import time, strftime, localtime

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


def main():
	cap = cv2.VideoCapture(0)
	
	# 720p : 2560x720 : 60 FPS
	# WVGA : 1344x376 : 100 FPS
	cap.set(5, 60)	# FPS
	cap.set(3, 1344)# Image Width
	cap.set(4, 376)	# Image Height
	cap.set(12, 0.8) # Saturation
	cap.set(10, 1.0) # Brightness
	
	frame_width = int(cap.get(3))
	frame_height = int(cap.get(4))
	print(frame_width, frame_height)
	

	VIDEO_RECORDING = True
		 
	if VIDEO_RECORDING:
		# Define the codec and create VideoWriter object.The output is stored in "videos/dual_image_video_D-M-Y_H-M-S.mp4" file.
		filename = 'videos/dual_image_video_'+ strftime("%d-%b-%Y_%H-%M-%S", localtime()) + '.avi'
		out = cv2.VideoWriter(filename,cv2.VideoWriter_fourcc('M','J','P','G'), 12, (int(frame_width/2),frame_height))

	while(cap.isOpened()):
		start = time()
		ret, img = cap.read()		
		if ret:			
			left_img = img[:frame_height, :int(frame_width/2)]
			right_img = img[:frame_height, int(frame_width/2):frame_width]
			if VIDEO_RECORDING:
				out.write(left_img)  
			cv2.imshow('image',left_img)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
		else:
			break
		print("Capture rate: {} Hz".format(1/float(time() - start)))
		print("Saturation: {}".format(cap.get(12)))
		print("Brightness: {}".format(cap.get(10)))
		print("-------------")
		
	cap.release()
	if VIDEO_RECORDING:
		out.release()
	cv2.destroyAllWindows()
	print("Video saved as: " + filename)
	
	
	
if __name__ == "__main__":
	main()
