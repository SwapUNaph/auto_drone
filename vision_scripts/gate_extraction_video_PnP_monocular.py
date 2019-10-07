#!/usr/bin/env python
import cv2
#import tf.transformations as tfs
import numpy as np
import math
from time import strftime, localtime, time
import matplotlib.pyplot as plt
import matplotlib.animation as animation


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




# MVA filter
class MVA:
    def __init__(self, n):
        self.n = n
        self.val_history = []
        self.filtered = []
        
    def update(self, val):
        if len(self.val_history) < self.n:
            self.val_history.append(val)
            return np.zeros(val.shape)
        else:
            if True :     #np.linalg.norm(np.subtract(self.filtered, val)) < 10:
                self.val_history.pop(0)
                self.val_history.append(val)
                self.filtered = np.sum(np.array(self.val_history), axis=0)/self.n
                return self.filtered
            else:
                return self.filtered
        

# rotate a vector by a quaternion
def qv_mult(q1, v1):
    #l = np.linalg.norm(v1)
    #v1 = tfs.unit_vector(v1)
    q2 = np.append(v1, 0.0)
    return tfs.quaternion_multiply(tfs.quaternion_multiply(q1, q2), tfs.quaternion_conjugate(q1))[:3]
    #return np.matmul(q1, np.matmul(v1, tfs.quaternion_conjugate(q1).T))

# Convert rotation vector to quaternion    
def rvec2quat(vector):
    theta = np.linalg.norm(vector)
    unit_vector = vector/theta
    sin_th_by_2 = math.sin(theta/2)
    x = unit_vector[0] * sin_th_by_2
    y = unit_vector[1] * sin_th_by_2
    z = unit_vector[2] * sin_th_by_2
    w = math.cos(theta/2)
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


def contourROIMean(contour, img):
    x,y,w,h = cv2.boundingRect(contour)
    return img[y:y+h, x:x+w].mean()

def annotateCorners(contour, img):
    contour = contour.reshape(4,2)
    center = map(int, contour.mean(axis=0))
    contour = contour.tolist()
    count = 1
    
    cv2.circle(img, (center[0], center[1]), 10, (0,255,255), 0)
    for point in contour:
        cv2.circle(img, (point[0], point[1]), 10, (0,255,255), 0)
        #cv2.putText(img, str(count), (point[0], point[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)
        count = count + 1
     
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
    blur = cv2.GaussianBlur(mask,(5,5), 3)
    cv2.imshow('Blur', blur)

    # Find contours
    contours, hierarchy = cv2.findContours(blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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
    quadrlFiltered = list(filter(lambda x: (cv2.contourArea(x) > 1000) , quadrl))
    #print("Contours after area filter: %d" % len(quadrlFiltered))

    # Filter for contour solidity > 0.9
    quadrlFiltered = list(filter(lambda x: (solidity(x) > 0.50) , quadrlFiltered))
    #print("Contours after solidity filter: %d" % len(quadrlFiltered))

    # Filter by contour aspect ratio: 1.20 > AR > 0.8
    quadrlFiltered = list(filter(lambda x: (aspectRatio(x) > 0.9) & (aspectRatio(x) < 1.11) , quadrlFiltered))
    #print("Contours after aspect ratio filter: %d" % len(quadrlFiltered))

    # Filter by contour mean
    quadrlFiltered = list(filter(lambda x: contourROIMean(x, blur) < 50 , quadrlFiltered))
    #print("Contours after aspect ratio filter: %d" % len(quadrlFiltered))

    #print("Square contour areas:")
    #for sq in quadrlFiltered:
        #print(cv2.contourArea(sq))

    # Sort quadrilaterals by area
    quadrlFiltered = sorted(quadrlFiltered, key=lambda x: cv2.contourArea(x))

    if len(quadrlFiltered) > 0:
        gate_contour = quadrlFiltered[0].reshape(4,2)
        
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


def rectifyStereoImage(img):
    
    # Camera Matrix 720p
    #cameraMatrix = np.array([[700, 0.0, 640], [0.0, 700, 360], [0.0, 0.0, 1.0]])
    
    # Distortion Coefficients 720p
    #distCoeffs = np.array([-0.1740500032901764, 0.028304599225521088, 0.0, 0.0, 0.0])
    
      # Camera Matrix
    cameraMatrix = np.array([[700.0499877929688, 0.0, 637.5999755859375],
                            [0.0, 700.0499877929688, 382.5060119628906],
                            [0.0, 0.0, 1.0]])
    
    # Distortion Coefficients
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
    
    
    
def getGatePose(contour, objectPoints):
    
    ##Camera Transformation wrt drone
    dRc = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    #dTc = np.array([[0,0,1,0],[-1,0,0, 0],[0,-1,0, 0], [0,0,0,1]])
    #dQuatC = tfs.quaternion_from_matrix(dTc)
    #dQuatC = [ 0.5, -0.5, 0.5, -0.5 ]
    
    ## Camera Matrix
    #cameraMatrix = np.array( [[ 258.58131479,    0. ,         348.1852167 ],
                              #[   0. ,         257.25344992,  219.07752178],
                              #[   0. ,           0.,            1.        ]]  )
    
    ## Distortion Coefficients
    #distCoeffs = np.array([[-0.36310169,  0.10981468,  0.0057042,  -0.001884,   -0.01328491]]  )
    
    # Camera Matrix 720p 
    cameraMatrix = np.array([[700, 0.0, 640], [0.0, 700, 360], [0.0, 0.0, 1.0]])

    # Distortion Coefficients 720p
    distCoeffs = np.array([-0.1740500032901764, 0.028304599225521088, 0.0, 0.0, 0.0])
    
    # Solve perspective 3 point algorithm
    (success, rvec, tvec) = cv2.solvePnP(objectPoints, contour.reshape(4,2).astype(float), cameraMatrix, distCoeffs)    
    rvec = np.squeeze(rvec)
    tvec = np.squeeze(tvec)
    
    quat = rvec2quat(rvec)
    
    # Transform to drone coordinates
    #tvec = qv_mult(dQuatC, tvec)    
    tvec = np.matmul(dRc, tvec.T)
    
    euler = np.matmul(dRc, rvec.T)
    euler = euler * 180.0 / math.pi
    #quat = tfs.quaternion_multiply(dQuatC, quat)
    #tvec = np.matmul(dRc, tvec.reshape(3,1))
    
    
    #print("Q : {}\nT: {}".format(quat, tvec))
    
    return (euler, tvec)

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

    gate_side = 1.4
    objectPoints = np.array([
            (-gate_side/2, -gate_side/2, 0.0),
            (-gate_side/2, gate_side/2, 0.0),
            (gate_side/2, gate_side/2, 0.0),
            (gate_side/2, -gate_side/2, 0.0)
        ])
    
    cap = cv2.VideoCapture("videos/avi_fast/blue_moving_fast.avi")
    #cap = cv2.VideoCapture(0)
    
    # 2.2K : 4416x1242 : 15 FPS
    # 1080p : 3840x1080 : 30 FPS
    # 720p : 2560x720 : 60 FPS
    # WVGA : 1344x376 : 100 FPS
    #cap.set(5, 30)    # FPS
    #cap.set(3, 640)# Image Width
    #cap.set(4, 480)    # Image Height
    #cap.set(12, 0.5) # Saturation
    #cap.set(10, 0.5) # Brightness
    
    frame_width = int(cap.get(3))
    frame_height = int(cap.get(4))
    print(frame_width, frame_height)
    
    VIDEO_RECORDING = False
         
    if VIDEO_RECORDING:
        # Define the codec and create VideoWriter object.The output is stored in 'outpy.avi' file.
        filename = 'videos/gate_detection_'+ strftime("%d-%b-%Y_%H-%M-%S", localtime()) + '.avi'
        out = cv2.VideoWriter(filename,cv2.VideoWriter_fourcc('M','J','P','G'), cap.get(5), (frame_width,frame_height))
    
    # Blue gate    
    #hsv_thresh_low = (110, 5, 100)
    #hsv_thresh_high = (150, 255, 255)
    
    # Green gate
    #hsv_thresh_low = (40, 10, 100)
    #hsv_thresh_high = (90, 255, 255)
    
    # Green gate rgb
    #hsv_thresh_low = (100, 200, 100)
    #hsv_thresh_high = (255, 255, 255)
    
    # Red gate
    #hsv_thresh_low = (130, 40, 30)
    #hsv_thresh_high = (200, 255, 255)
    
    # Orange gate
    #hsv_thresh_low = (0, 100, 100)
    #hsv_thresh_high = (40, 255, 255)
    
    # Orange gate BGR
    #hsv_thresh_low = (92, 92, 157)
    #hsv_thresh_high = (196, 202, 206)
    
    #All color LEDs : 0 0 191 180 255 255
    hsv_thresh_low = (0, 0, 0)
    hsv_thresh_high = (180, 150, 100)
    
    # MVA filters
    orntFilter = MVA(15)
    trnslFilter = MVA(15)
    
    # plotting variables
    frame_number = 0
    frame_index = []
    xs = []
    ys = []
    zs = []
    

    while(cap.isOpened()):
        start = time()    
        ret, img = cap.read()    
        
        
        if ret:
            frame_number = frame_number + 1
            #height, width, depth = img.shape
            #half_width = int(width/2)
            
            
            # Separate left and right images
            #left_img = img[:, :half_width]
            #right_img = img[:, half_width:width]
            
            ## Rectify images
            #left_img = cv2.undistort(left_img, cameraMatrix, distCoeffs)
            #right_img = cv2.undistort(right_img, cameraMatrix, distCoeffs)
            #img = np.concatenate((left_img, right_img), axis=1)
            
            left_cnt = detect_gate(img, hsv_thresh_low, hsv_thresh_high)
            #right_cnt = detect_gate(right_img, hsv_thresh_low, hsv_thresh_high)
            
            
            if left_cnt is not None and np.array_equal(left_cnt.shape, [4,2]):    
                
                euler, tvec = getGatePose(left_cnt, objectPoints)
                
                tvec = trnslFilter.update(tvec)
                print("X:\t{0:.2f}, Y:\t{1:.2f}, Z:\t{2:.2f} (m)".format(tvec[0], tvec[1], tvec[2]))

                euler = orntFilter.update(euler)
                print("Roll:\t{0:.2f}, Pitch:\t{1:.2f}, Yaw:\t{2:.2f} (deg)".format(euler[0], euler[1], euler[2]))
                
                cv2.drawContours(img, [left_cnt.reshape(4,2)], 0, (0,0,255), 1)
                #cv2.circle(img, (left_center[0], left_center[1]), 10, (255,255,0), 0)
                cv2.putText(img, "X: {0:.2f}, Y: {1:.2f}, Z: {2:.2f} (m)".format(tvec[0], tvec[1], tvec[2]), (10, frame_height - 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2, cv2.LINE_AA)
                cv2.putText(img, "Roll: {0:.2f}, Pitch: {1:.2f}, Yaw: {2:.2f} (deg)".format(euler[0], euler[1], euler[2]), (10, frame_height - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2, cv2.LINE_AA)
            
                cv2.putText(img, "X: {0:.2f}, Y: {1:.2f}, Z: {2:.2f} (m)".format(tvec[0], tvec[1], tvec[2]), (10, frame_height - 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(img, "Roll: {0:.2f}, Pitch: {1:.2f}, Yaw: {2:.2f} (deg)".format(euler[0], euler[1], euler[2]), (10, frame_height - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 1, cv2.LINE_AA)

                annotateCorners(left_cnt, img)  
                
                '''
                # Plot graph
                frame_index.append(frame_number)
                xs.append(tvec[0])  
                ys.append(tvec[1])
                zs.append(tvec[2])
                    
                plt.plot(frame_index, xs, c='red') 
                plt.plot(frame_index, ys, c='green') 
                plt.plot(frame_index, zs, c='blue')
                plt.draw()
                plt.pause(0.0001) 
                '''
                    
            print("Detection rate: {} Hz".format(1/float(time() - start)))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            if VIDEO_RECORDING:
                out.write(img)  
            print("FPS: {} Hz".format(cap.get(5)))
            cv2.imshow('Gate detection image',img)
            #cv2.imshow('right image',right_img)q

        else:
            break
            
    cap.release()
    if VIDEO_RECORDING:
        out.release()
    cv2.destroyAllWindows()




if __name__ == '__main__':
    main()
