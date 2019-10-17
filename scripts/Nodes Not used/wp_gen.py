#!/usr/bin/python

import rospy
import signal
import numpy as np
import math
import time
from tf import transformations
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from auto_drone.msg import WP_Msg



def signal_handler(_, __):
    # enable Ctrl+C of the script
    sys.exit(0)



if __name__ == '__main__':
	# Enable killing the script with Ctrl+C.
	signal.signal(signal.SIGINT, signal_handler)

	# initialize node
	rospy.init_node('wp_generator', anonymous=False)




	publisher_wp_gen = rospy.Publisher("/auto/filtered_gate_WP",  WP_Msg, queue_size=1)



	dist = 2

	theta = 0
	step = .025
	limit = 45.0*np.pi/180
	dt = .05

	gate_hdg = 0.0

	while True:
		print theta
		if abs(theta) < limit:
			theta = theta + step
		else:
			step = -1 * step
			theta = theta + step

		wp_msg = WP_Msg()
		wp_msg.pos.y = dist * math.sin(theta+gate_hdg)
		wp_msg.pos.x = dist * math.cos(theta+gate_hdg)
		wp_msg.pos.z = 0
		wp_msg.hdg = theta

		publisher_wp_gen.publish(wp_msg)
		time.sleep(dt)


