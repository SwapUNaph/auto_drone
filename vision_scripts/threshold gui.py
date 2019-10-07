from __future__ import print_function
import cv2

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

    frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame_threshold = cv2.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))

    cv2.imshow(window_capture_name, frame)
    cv2.imshow(window_detection_name, frame_threshold)

    # if frame_count == cap.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT):
    #     frame_count = 0  # Or whatever as long as it is the same as next line
    #     cap.set(cv2.cv.CV_CAP_PROP_POS_FRAMES, 0)

    key = cv2.waitKey(100)
    if key == ord('q') or key == 27:
        break
