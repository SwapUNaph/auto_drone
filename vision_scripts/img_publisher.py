#!/usr/bin/env python
import rospy
import signal
import sys
import cv2
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def signal_handler(_, __):
    sys.exit(0)



if __name__ == '__main__':

    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('image_processing', anonymous=False)

    cap = cv2.VideoCapture(0)

    cap.set(3,2560)
    cap.set(4,720)
    cap.set(5,60)

    publisher_gate = rospy.Publisher('/visual/test_hough', Image, queue_size=1,latch=True)
    bridge = CvBridge()


    while True:
        ret, img2 = cap.read()
	if img2 is not None:
	    img = img2[:,0:1280,:]

            img_msg = bridge.cv2_to_imgmsg(img,encoding='rgb8')
            publisher_gate.publish(img_msg)
	    print img.shape
