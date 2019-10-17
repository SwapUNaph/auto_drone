# !/usr/bin/env python
import numpy as np
import cv2
import time



def main():
	# termination criteria
	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

	# Number of Corners and Square side
	nx = 9
	ny = 6
	squareSide = 1

	# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
	objp = np.zeros((ny*nx,3), np.float32)
	objp[:,:2] = np.mgrid[0:nx,0:ny].T.reshape(-1,2)*squareSide

	#print(objp)

	# Arrays to store object points and image points from all the images.
	objpoints = [] # 3d point in real world space
	imgpoints = [] # 2d points in image plane.

	#images = glob.glob('*.jpg')

	cap = cv2.VideoCapture(0)
	cap.set(3, 640)
	cap.set(4, 480)

	print("Press 'd' once you got enough images.")
	print("Press 'q' to cancel and exit.")

	while True:
		_, img = cap.read()
		gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

		# Find the chess board corners
		ret, corners = cv2.findChessboardCorners(gray, (nx,ny),None)
		#print(time.localtime())
		# If found, add object points, image points (after refining them)

		#cv2.imshow('img',img)
		if ret == True:
		    objpoints.append(objp)

		    corners2 = cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),criteria)
		    imgpoints.append(corners2)

		    # Draw and display the corners
		    img = cv2.drawChessboardCorners(img, (nx,ny), corners2,ret)
		    cv2.imshow('img',img)
		    print("Number of points: %d" % len(objpoints))


		key = cv2.waitKey(500)
		
		if key == ord('d') or key == ord('D'):
		    break
		    
		elif key == ord('q') or key == ord('Q'):
			print("Quiting calibration without saving.")
			return
		    
		 


	cv2.destroyAllWindows()

	# get camera matrices and distortion coeefs and save them in a file
	print("Calibrating camera....")
	ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
	print("Done Calibration")
	print("Saving calibration....")

	cameraCaliName = "cameraCalibration.txt" 
	with open(cameraCaliName, "w") as fh:
		print("K: {} \n".format(mtx))
		fh.write("K: {} \n".format(mtx))
		print("D: {} \n".format(dist))
		fh.write("D: {} \n".format(dist))


	print("Done Saving Calibration!")

	
if __name__ == '__main__':
	main()

