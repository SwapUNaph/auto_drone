#!/usr/bin/python
'''
Program description: This node simulates camera node
Author: Swapneel Naphade (snaphade@umd.edu)
version: 2.0
Date: 9/20/19
Change log: 

'''

import rospy
import signal
import sys
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

global bridge
bridge = CvBridge()

global cap
cap = cv2.VideoCapture(0)
cap.set(3, 640) # Width
cap.set(4, 480) # Height
cap.set(5, 60) # FPS

def signal_handler(_, __):
    # enable Ctrl+C of the script
    sys.exit(0)

if __name__ == '__main__':

    signal.signal(signal.SIGINT, signal_handler)

    # Set simulation parameter to True. The system will start in simulation mode by default.
    SIMULATION = rospy.get_param("/simulation")
        
    # Initialize node
    rospy.init_node('camera', anonymous=False)

    # Publishers
    # Image publication
    image_publisher = rospy.Publisher('/autodrone/image_raw', Image, queue_size=1)

    # Update rate for the camera loop
    rate = rospy.Rate(50) # 50 hz

    while not rospy.is_shutdown():
        ret, img = cap.read()
        if img is None:
            rospy.logerr("No Camera detected!!!")
            rate.sleep()
        else:
            w = cap.get(3)
            #img = img[:,:int(w/2)]
            image_publisher.publish(bridge.cv2_to_imgmsg(img[:, :int(w/2)],"bgr8"))
            #rospy.loginfo("[Camera] FPS: %d, Width: %d, Height: %d".format(cap.get(5), cap.get(3), cap.get(4)))	
       
	
