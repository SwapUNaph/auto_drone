from __future__ import print_function
import cv2
import numpy as np

folder_name = "videos/"
filename = "threshold.txt"
LIVE = True

max_value = 255
max_value_H = 180
low_H = 0
low_S = 0
low_V = 0
high_H = max_value_H
high_S = max_value
high_V = max_value


######################  Detection Parameters  ##########################

# Gate size in meters
GATE_SIZE = 1.20
NOT_GATE_SIZE = 0.8


# HSV thresholds for LED gate
#hsv_thresh_low = (0, 0, 250)
#hsv_thresh_high = (180, 20, 255)

# Gate thresholds
AREA_THRESH = 1000
ASPECT_RATIO_THRESH_LOW = 0.7 # Should be between 0.0 and 1.0
ASPECT_RATIO_THRESH_HIGH = 1/ASPECT_RATIO_THRESH_LOW
SOLIDITY_THRESH = 0.90
ROI_MEAN_THRESH = 100

DETECTION_ACTIVE = True
GATE_TYPE_VERTICAL = None

########################################################################

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


def detect_gate(img, hsv_thresh_low, hsv_thresh_high):
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
    blur = cv2.GaussianBlur(mask,(7,7), 5)
    #cv2.imshow('Blur', blur)

    # Find contours
    contours, hierarchy = cv2.findContours(blur, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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
        gate_contour = quadrlFiltered[-1].reshape(4,2)
        
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
	x,y,w,h = cv2.boundingRect(gate_cnt_sorted_np)
	
	right_x = x + w
	left_x = x - w
	print 
	
	if y-40 > 0 and y+h+40 < img.shape[0] and x-40 > 0 and x+h+40 < img.shape[1]:
	    left_img = mask[y-30:y+h+30, left_x-30:left_x+w+30]
	    right_img = mask[y-30:y+h+30, right_x-30:right_x+w+30]


	else:
	    left_img = None
	    right_img = None
        return gate_cnt_sorted_np, left_img, right_img
    else:
        #print("No gate detected!")
        return None, None, None

with open(filename, 'r') as th_file:
    vals = th_file.readlines()[0].split(" ")
    low_H = int(vals[0])
    low_S = int(vals[1])
    low_V = int(vals[2])
    high_H = int(vals[3])
    high_S = int(vals[4])
    high_V = int(vals[5])
    print("HSV Values loaded from %s : %d %d %d %d %d %d" % (filename, low_H, low_S, low_V, high_H, high_S, high_V))

window_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'
low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'
save_bar = "Save OFF : 0 /n Save OFF : 1"


def on_low_H_thresh_trackbar(val):
    global low_H
    global high_H
    low_H = val
    low_H = min(high_H - 1, low_H)
    cv2.setTrackbarPos(low_H_name, window_detection_name, low_H)


def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    high_H = max(high_H, low_H + 1)
    cv2.setTrackbarPos(high_H_name, window_detection_name, high_H)


def on_low_S_thresh_trackbar(val):
    global low_S
    global high_S
    low_S = val
    low_S = min(high_S - 1, low_S)
    cv2.setTrackbarPos(low_S_name, window_detection_name, low_S)


def on_high_S_thresh_trackbar(val):
    global low_S
    global high_S
    high_S = val
    high_S = max(high_S, low_S + 1)
    cv2.setTrackbarPos(high_S_name, window_detection_name, high_S)


def on_low_V_thresh_trackbar(val):
    global low_V
    global high_V
    low_V = val
    low_V = min(high_V - 1, low_V)
    cv2.setTrackbarPos(low_V_name, window_detection_name, low_V)


def on_high_V_thresh_trackbar(val):
    global low_V
    global high_V
    high_V = val
    high_V = max(high_V, low_V + 1)
    cv2.setTrackbarPos(high_V_name, window_detection_name, high_V)


def on_save(val):
    if val == 1:
        with open(filename, 'w') as th_file:
            th_file.write("%d %d %d %d %d %d" % (low_H, low_S, low_V, high_H, high_S, high_V))
	    print("HSV Values saved to %s : %d %d %d %d %d %d" % (filename, low_H, low_S, low_V, high_H, high_S, high_V))



video_file = "avi_staying/blue_5m.avi"
image_file = "vlcsnap-2019-09-12-17h25m19s944.png"

if LIVE:
	#cap = cv2.VideoCapture(folder_name + video_file)
	cap = cv2.VideoCapture(0)
else:
	frame = cv2.imread("vlcsnap-2019-09-12-17h25m19s944.png")

cv2.namedWindow(window_capture_name)
cv2.namedWindow(window_detection_name)
cv2.createTrackbar(low_H_name, window_detection_name, low_H, max_value_H, on_low_H_thresh_trackbar)
cv2.createTrackbar(high_H_name, window_detection_name, high_H, max_value_H, on_high_H_thresh_trackbar)
cv2.createTrackbar(low_S_name, window_detection_name, low_S, max_value, on_low_S_thresh_trackbar)
cv2.createTrackbar(high_S_name, window_detection_name, high_S, max_value, on_high_S_thresh_trackbar)
cv2.createTrackbar(low_V_name, window_detection_name, low_V, max_value, on_low_V_thresh_trackbar)
cv2.createTrackbar(high_V_name, window_detection_name, high_V, max_value, on_high_V_thresh_trackbar)
cv2.createTrackbar(save_bar, window_detection_name, 0, 1, on_save)

frame_count = 0
while True:
    if LIVE: 
	ret, frame = cap.read()
	frame_count = frame_count + 1
	if frame is None:
	    break

    frame = frame[:,:int(cap.get(3)/2)]
    frame = cv2.resize(frame, None, fx=0.5, fy=0.5) 
    
    frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame_threshold = cv2.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))
    # Blur 
    blur = cv2.GaussianBlur(frame_threshold,(7,7), 5)

    gate, left_img, right_img = detect_gate(frame, (low_H, low_S, low_V), (high_H, high_S, high_V))
            
    # If gate detected succesfully
    if gate is not None and np.array_equal(gate.shape, [4,2]):
        cv2.drawContours(frame, [gate.reshape(4,2)], 0, (255, 0, 0), 2)
        annotateCorners(gate, frame)  
                
    cv2.imshow(window_capture_name, frame)
    if left_img is not None:
	cv2.imshow("left_window", left_img)
	cv2.imshow("right window", right_img)
    cv2.imshow(window_detection_name, blur)

    key = cv2.waitKey(100)
    if key == ord('q') or key == 27:
        break
