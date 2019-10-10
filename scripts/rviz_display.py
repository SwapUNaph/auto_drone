#!/usr/bin/env python

# Script developed by Derek Thompson
#  --- Changelog ---
# Goal:     Display all relevant flight data like position, waypoint, state, video, battery level, wifi status
# Status:   06/19: There is a gui that displays some information.
#           11/03: Fully functional, comments still missing

import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped, PointStamped, Point
from std_msgs.msg import Empty, Int32, Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
# from bebop_auto.msg import Gate_Detection_Msg, WP_Msg, Auto_Driving_Msg
from adr2019.msg import Gate_Detection_Msg, WP_Msg, Auto_Driving_Msg
from bebop_msgs.msg import Ardrone3CameraStateOrientation
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged
from bebop_msgs.msg import CommonCommonStateWifiSignalChanged
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
from bebop_msgs.msg import Ardrone3PilotingStateAttitudeChanged
from bebop_msgs.msg import Ardrone3PilotingStateSpeedChanged
#from bebop_msgs.msg import OverHeatStateOverHeatChanged

from visualization_msgs.msg import Marker, MarkerArray

import cv2
import numpy as np
import time
import math
import signal
import sys
import tf

def signal_handler(signal, frame):
    sys.exit(0)

class bebop_data:
    def __init__(self):        

        self.vichile_pub = rospy.Publisher("/auto/rviz/vehicle", MarkerArray, queue_size=1)
	
        self.gate_pub = rospy.Publisher("/auto/rviz/gate", MarkerArray, queue_size=1)

        self.arena_pub = rospy.Publisher("/auto/rviz/arena", MarkerArray, queue_size=1)

        self.gate_number = 0
        self.state_level = 9
        self.gate_arm_theta = 0.0

        self.battery_level = 0
        self.gate_size = 1.4
        
        # self.gate_visual_pub = rospy.Publisher("/auto/rviz/gate_visual", MarkerArray, queue_size=1)
        # self.gate_blind_pub = rospy.Publisher("/auto/rviz/gate_blind", MarkerArray, queue_size=1)
        # self.gate_current = rospy.Publisher("/auto/rviz/gate_current", MarkerArray, queue_size=1)
        # self.gate_look_pub = rospy.Publisher("/auto/rviz/gate_look", MarkerArray, queue_size=1)
        # self.command_viz_pub = rospy.Publisher("/auto/rviz/commands", MarkerArray, queue_size=1)


        # static_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period_in_ms
        self.tbr = tf.TransformBroadcaster()



        rospy.Subscriber("/bebop/camera_control", Twist, self.callback, "cam_ctrl")
        rospy.Subscriber("/bebop/reset", Empty, self.callback, "reset")
        rospy.Subscriber("/zed/odom", Odometry, self.callback, "zed_odom")
        rospy.Subscriber("/bebop/odom", Odometry, self.callback, "odom")
        rospy.Subscriber("/auto/auto_drive", Auto_Driving_Msg, self.callback, "cmds")
        rospy.Subscriber("/bebop/states/ardrone3/CameraState/Orientation", Ardrone3CameraStateOrientation, self.callback,"cam_orient")
        rospy.Subscriber("/bebop/states/common/CommonState/BatteryStateChanged", CommonCommonStateBatteryStateChanged,self.callback, "battery")
        rospy.Subscriber("/bebop/states/common/CommonState/WifiSignalChanged", CommonCommonStateWifiSignalChanged,self.callback, "wifi")
        rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged", Ardrone3PilotingStateAltitudeChanged,self.callback, "altitude")
        rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AttitudeChanged", Ardrone3PilotingStateAttitudeChanged,self.callback, "attitude")
        # rospy.Subscriber("/bebop/states/ardrone3/PilotingState/PositionChanged", Ardrone3PilotingStatePositionChanged,  callback, "position") no updates
        rospy.Subscriber("/bebop/states/ardrone3/PilotingState/SpeedChanged", Ardrone3PilotingStateSpeedChanged,self.callback, "speed")

        # rospy.Subscriber("/bebop/states/common/OverHeatState/OverHeatChanged",   OverHeatStateOverHeatChanged,          callback, "overheat")
        # ~states/enable_pilotingstate_flyingstatechanged parameter to true
        # enable the publication of flying state changes to topic states/ARDrone3/PilotingState/FlyingStateChanged

        # rospy.Subscriber("/auto/gate_detection_gate", Image,self.callback, 'Image')
        rospy.Subscriber("/bebop/image_raw", Image,self.callback, 'Image')
        rospy.Subscriber("/auto/wp_visual", WP_Msg, self.callback,'wp_visual')
        rospy.Subscriber("/auto/wp_current", WP_Msg, self.callback,'wp_current')
        rospy.Subscriber("/auto/wp_blind", WP_Msg, self.callback,'wp_blind')
        rospy.Subscriber("/auto/wp_look", WP_Msg, self.callback,'wp_look')
        rospy.Subscriber("/auto/state_auto", Int32, self.callback,'state')
        # rospy.Subscriber("/auto/state_auto", Float32, self.callback,'state')

        rospy.Subscriber("/auto/pose", Pose, self.callback,'vehicle_pos')

        

        rospy.Subscriber("/gate_pose", Pose, self.callback,'gate_pose')

        rospy.Subscriber("/auto/state", Int32,self.callback, "state")      

        # self.publish_arena()  


    def publish_arena(self):

    	marker_array = MarkerArray()  

    	quat1 = tf.transformations.quaternion_from_euler(0, 0, 3.1415/2)
    	quat2 = tf.transformations.quaternion_from_euler(0, 0, -3.1415/2)
    	marker_array.markers.extend(self.gate_marker('gate_frame11','gate11').markers)
    	marker_array.markers.extend(self.gate_marker('gate_frame12','gate12').markers)
    	marker_array.markers.extend(self.gate_marker('gate_frame21','gate21').markers)
    	marker_array.markers.extend(self.gate_marker('gate_frame22','gate22').markers)

        # marker_array = self.gate_marker_1('gate_frame11')

        arena_rate = rospy.Rate(.25)
        while True:
            print 'published arena'
            self.tbr.sendTransform((0.7, 0.0, 3.1),(quat1[0],quat1[1],quat1[2],quat1[3]),rospy.Time.now(),'gate_frame11', "track_frame")
            self.tbr.sendTransform((0.7, 0.0, 1.7),(quat1[0],quat1[1],quat1[2],quat1[3]),rospy.Time.now(),'gate_frame12', "track_frame")
            self.tbr.sendTransform((-0.7, 13.0, 1.7),(quat2[0],quat2[1],quat2[2],quat2[3]),rospy.Time.now(),'gate_frame21', "track_frame")
            self.tbr.sendTransform((-2.1, 13.0, 1.7),(quat2[0],quat2[1],quat2[2],quat2[3]),rospy.Time.now(),'gate_frame22', "track_frame")
            self.arena_pub.publish(marker_array)
            arena_rate.sleep()


    def gate_marker(self,frame,gate_number):

        marker_array = MarkerArray()

        gate_marker_1 = Marker()
        gate_marker_1.header.frame_id = frame
        gate_marker_1.header.stamp    = rospy.Time.now()
        gate_marker_1.ns = gate_number
        gate_marker_1.id = 1
        gate_marker_1.type = 1
        gate_marker_1.action = 0
        gate_marker_1.pose.position.z = self.gate_size/2
        gate_marker_1.pose.orientation.w = 1
        gate_marker_1.scale.x = .05
        gate_marker_1.scale.y = self.gate_size
        gate_marker_1.scale.z = .05
        gate_marker_1.color.r = 1.0
        gate_marker_1.color.g = .5
        gate_marker_1.color.b = 0.0
        gate_marker_1.color.a = 1.0
        gate_marker_1.lifetime = rospy.Duration(0)
        marker_array.markers.append(gate_marker_1)
        
        gate_marker_2 = Marker()
        gate_marker_2.header.frame_id = frame
        gate_marker_2.header.stamp    = rospy.Time.now()
        gate_marker_2.ns = gate_number
        gate_marker_2.id = 2
        gate_marker_2.type = 1
        gate_marker_2.action = 0
        gate_marker_2.pose.position.y = self.gate_size/2
        gate_marker_2.pose.position.z = 0
        gate_marker_2.pose.orientation.w = 1
        gate_marker_2.scale.x = .05
        gate_marker_2.scale.y = .04
        gate_marker_2.scale.z = self.gate_size
        gate_marker_2.color.r = 1.0
        gate_marker_2.color.g = .5
        gate_marker_2.color.b = 0.0
        gate_marker_2.color.a = 1.0
        gate_marker_2.lifetime = rospy.Duration(0)
        marker_array.markers.append(gate_marker_2)

        gate_marker_3 = Marker()
        gate_marker_3.header.frame_id = frame
        gate_marker_3.header.stamp    = rospy.Time.now()
        gate_marker_3.ns = gate_number
        gate_marker_3.id = 3
        gate_marker_3.type = 1
        gate_marker_3.action = 0
        gate_marker_3.pose.position.y = -self.gate_size/2
        gate_marker_3.pose.orientation.w = 1
        gate_marker_3.scale.x = .05
        gate_marker_3.scale.y = .05
        gate_marker_3.scale.z = self.gate_size
        gate_marker_3.color.r = 1.0
        gate_marker_3.color.g = .5
        gate_marker_3.color.b = 0.0
        gate_marker_3.color.a = 1.0
        gate_marker_3.lifetime = rospy.Duration(0)
        marker_array.markers.append(gate_marker_3)

        gate_marker_4 = Marker()
        gate_marker_4.header.frame_id = frame
        gate_marker_4.header.stamp    = rospy.Time.now()
        gate_marker_4.ns = gate_number
        gate_marker_4.id = 4
        gate_marker_4.type = 1
        gate_marker_4.action = 0
        gate_marker_4.pose.position.z = -self.gate_size/2
        gate_marker_4.pose.orientation.w = 1
        gate_marker_4.scale.x = .05
        gate_marker_4.scale.y = self.gate_size
        gate_marker_4.scale.z = .05
        gate_marker_4.color.r = 1.0
        gate_marker_4.color.g = .5
        gate_marker_4.color.b = 0.0
        gate_marker_4.color.a = 1.0
        gate_marker_4.lifetime = rospy.Duration(0)
        marker_array.markers.append(gate_marker_4)

        
        # marker_array.markers.append(gate_marker_n)            
        
        return marker_array


        


    def callback(self,data,args):
        # rospy.loginfo(rospy.get_caller_id() + "\nI heard %s", data)
        if args == "gate_gate":
            self.gate_size = data.data
        
        elif args == "reset":
            # print("reset pressed")
            pass

        elif args == "state":
            if data.data != self.state_level:
                self.gate_number = self.gate_number+1
                self.state_level = data.data


        elif args == 'vehicle_pos':
            quat = data.orientation
            # print quat
            pos = data.position
            # quat = tf.transformations.quaternion_from_euler(quat.x,quat.y,quat.z,quat.w)
            
            
            marker_array = MarkerArray()

            point_marker = Marker()
            point_marker.header.frame_id = "vehicle_frame"
            point_marker.header.stamp    = rospy.get_rostime()
            point_marker.ns = "vehicle"
            point_marker.id = 0
            point_marker.type = 1 # prism
            point_marker.action = 0
            point_marker.scale.x = .3
            point_marker.scale.y = .12
            point_marker.scale.z = .08
            point_marker.pose.orientation.w = 1
            point_marker.color.r = 0
            point_marker.color.g = 0
            point_marker.color.b = 1
            point_marker.color.a = 1.0
            point_marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(point_marker)

            # point_marker2 = Marker()
            # point_marker2.header.frame_id = "vehicle_frame"
            # point_marker2.header.stamp    = rospy.get_rostime()
            # point_marker2.ns = "vehicle"
            # point_marker2.action = 0
            # point_marker2.type = 3
            # point_marker2.id = 1
            # point_marker2.scale.x = .12
            # point_marker2.scale.y = .12
            # point_marker2.scale.z = .02
            # point_marker2.pose.position.z = .02
            # point_marker2.pose.position.x = .1
            # point_marker2.pose.position.y = .1
            # point_marker2.pose.orientation.w = 1
            # point_marker2.color.r = .5
            # point_marker2.color.g = .5
            # point_marker2.color.b = .5
            # point_marker2.color.a = 1.0
            # point_marker2.lifetime = rospy.Duration(0)
            # marker_array.markers.append(point_marker2)


            # point_marker3 = Marker()
            # point_marker3.header.frame_id = "vehicle_frame"
            # point_marker3.header.stamp    = rospy.get_rostime()
            # point_marker3.ns = "vehicle"
            # point_marker3.color.r = .5
            # point_marker3.color.g = .5
            # point_marker3.color.b = .5
            # point_marker3.color.a = 1.0
            # point_marker3.type = 3
            # point_marker3.id = 2
            # point_marker3.pose.position.z = .02
            # point_marker3.pose.position.x = .1
            # point_marker3.pose.position.y = -.1
            # point_marker3.pose.orientation.w = 1
            # point_marker3.scale.x = .12
            # point_marker3.scale.y = .12
            # point_marker3.scale.z = .02
            # point_marker3.lifetime = rospy.Duration(0)
            # marker_array.markers.append(point_marker3)

            # point_marker4 = Marker()
            # point_marker4.header.frame_id = "vehicle_frame"
            # point_marker4.header.stamp    = rospy.get_rostime()
            # point_marker4.ns = "vehicle"
            # point_marker4.color.r = .5
            # point_marker4.color.g = .5
            # point_marker4.color.b = .5
            # point_marker4.color.a = 1.0
            # point_marker4.type = 3
            # point_marker4.id = 3
            # point_marker4.pose.position.z = .02
            # point_marker4.pose.position.x = -.1
            # point_marker4.pose.position.y = -.1
            # point_marker4.pose.orientation.w = 1
            # point_marker4.scale.x = .12
            # point_marker4.scale.y = .12
            # point_marker4.scale.z = .02
            # point_marker4.lifetime = rospy.Duration(0)
            # marker_array.markers.append(point_marker4)

            # point_marker5 = Marker()
            # point_marker5.header.frame_id = "vehicle_frame"
            # point_marker5.header.stamp    = rospy.get_rostime()
            # point_marker5.ns = "vehicle"
            # point_marker5.color.r = .5
            # point_marker5.color.g = .5
            # point_marker5.color.b = .5
            # point_marker5.color.a = 1.0
            # point_marker5.type = 3
            # point_marker5.id = 4
            # point_marker5.pose.position.z = .02
            # point_marker5.pose.position.x = -.1
            # point_marker5.pose.position.y = .1
            # point_marker5.pose.orientation.w = 1
            # point_marker5.scale.x = .12
            # point_marker5.scale.y = .12
            # point_marker5.scale.z = .02
            # point_marker5.lifetime = rospy.Duration(0)
            # marker_array.markers.append(point_marker5)
            
            if (quat.x + quat.y + quat.z + quat.w) == 0:
                quat.w = 1

            self.tbr.sendTransform((pos.x,pos.y,pos.z),(quat.x,quat.y,quat.z,quat.w),rospy.get_rostime(),'vehicle_frame', "track_frame")

            self.vichile_pub.publish(marker_array)
            
            

        elif args == "odom":
            return
            quat = data.pose.pose.orientation
            # print quat
            pos = data.pose.pose.position
            # quat = tf.transformations.quaternion_from_euler(quat.x,quat.y,quat.z,quat.w)

            
            marker_array = MarkerArray()

            point_marker = Marker()
            point_marker.header.frame_id = "vehicle_frame"
            point_marker.header.stamp    = rospy.get_rostime()
            point_marker.ns = "vehicle"
            point_marker.id = 0
            point_marker.type = 1 # prism
            point_marker.action = 0
            point_marker.scale.x = .2
            point_marker.scale.y = .05
            point_marker.scale.z = .05
            point_marker.pose.orientation.w = 1
            point_marker.color.r = 0
            point_marker.color.g = 0
            point_marker.color.b = 1
            point_marker.color.a = 1.0
            point_marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(point_marker)

            point_marker2 = Marker()
            # point_marker2.header.frame_id = "vehicle_frame"
            point_marker2.header.stamp    = rospy.get_rostime()
            point_marker2.ns = "vehicle"
            point_marker2.action = 0
            point_marker2.type = 3
            point_marker2.id = 1
            point_marker2.scale.x = .02
            point_marker2.scale.y = .12
            point_marker2.scale.z = .12
            point_marker2.pose.position.z = .02
            point_marker2.pose.position.x = .1
            point_marker2.pose.position.y = .1
            point_marker2.pose.orientation.w = 1
            point_marker2.color.r = .5
            point_marker2.color.g = .5
            point_marker2.color.b = .5
            point_marker2.color.a = 1.0
            point_marker2.lifetime = rospy.Duration(0)
            marker_array.markers.append(point_marker2)

            point_marker3 = Marker()
            # point_marker3.header.frame_id = "vehicle_frame"
            point_marker3.header.stamp    = rospy.get_rostime()
            point_marker3.ns = "vehicle"
            point_marker3.color.r = .5
            point_marker3.color.g = .5
            point_marker3.color.b = .5
            point_marker3.color.a = 1.0
            point_marker3.type = 3
            point_marker3.id = 2
            point_marker3.pose.position.z = .02
            point_marker3.pose.position.x = .1
            point_marker3.pose.position.y = -.1
            point_marker3.pose.orientation.w = 1
            point_marker3.scale.x = .02
            point_marker3.scale.y = .12
            point_marker3.scale.z = .12
            point_marker3.lifetime = rospy.Duration(0)
            marker_array.markers.append(point_marker3)

            point_marker4 = Marker()
            # point_marker4.header.frame_id = "vehicle_frame"
            point_marker4.header.stamp    = rospy.get_rostime()
            point_marker4.ns = "vehicle"
            point_marker4.color.r = .5
            point_marker4.color.g = .5
            point_marker4.color.b = .5
            point_marker4.color.a = 1.0
            point_marker4.type = 3
            point_marker4.id = 3
            point_marker4.pose.position.z = .02
            point_marker4.pose.position.x = -.1
            point_marker4.pose.position.y = -.1
            point_marker4.pose.orientation.w = 1
            point_marker4.scale.x = .02
            point_marker4.scale.y = .12
            point_marker4.scale.z = .12
            point_marker4.lifetime = rospy.Duration(0)
            marker_array.markers.append(point_marker4)

            point_marker5 = Marker()
            # point_marker5.header.frame_id = "vehicle_frame"
            point_marker5.header.stamp    = rospy.get_rostime()
            point_marker5.ns = "vehicle"
            point_marker5.color.r = .5
            point_marker5.color.g = .5
            point_marker5.color.b = .5
            point_marker5.color.a = 1.0
            point_marker5.type = 3
            point_marker5.id = 4
            point_marker5.pose.position.z = .02
            point_marker5.pose.position.x = -.1
            point_marker5.pose.position.y = .1
            point_marker5.pose.orientation.w = 1
            point_marker5.scale.x = .02
            point_marker5.scale.y = .12
            point_marker5.scale.z = .12
            point_marker5.lifetime = rospy.Duration(0)
            marker_array.markers.append(point_marker5)

            self.tbr.sendTransform((pos.x,pos.y,pos.z),(quat.x,quat.y,quat.z,quat.w),rospy.get_rostime(),'vehicle_frame', "track_frame")
            self.vichile_pub.publish(marker_array)
            
            
            
            
        elif args == "battery":

            # print ' '
            # print 'Battery level: ',data.percent
            # print ' '
            self.battery_level = data.percent
        elif args == "wifi":
            if data.rssi * 2 + 160 < 100.0:
                print 'R: ',(data.rssi * 2 + 160),'  B: ',self.battery_level
            else:
                print 'R: ',(data.rssi * 2 + 160),' B: ',self.battery_level
        

        elif args == "altitude":
            # print "altitude ",data.altitude
            pass
        
        elif args == "attitude":
            # print "attitude", print [data.roll, data.pitch, data.yaw]
            pass
        
        elif args == "speed":
            # print "speed", [data.speedX, data.speedY, data.speedZ]
            pass
        
        elif args == "overheat":
            print("overheat")
            # print data
        
        elif args == 'cmds':
            
            marker_array = MarkerArray()

            gate_marker_1 = Marker()
            gate_marker_1.header.frame_id = "vehicle_frame"
            gate_marker_1.header.stamp    = rospy.get_rostime()
            gate_marker_1.ns = "effort"
            gate_marker_1.id = 0
            gate_marker_1.type = 1
            gate_marker_1.action = 0
            gate_marker_1.pose.orientation.w = 1
            gate_marker_1.pose.position.x = data.x*5/2.0
            gate_marker_1.scale.x = data.x*5
            gate_marker_1.scale.y = .04
            gate_marker_1.scale.z = .04
            gate_marker_1.color.r = 1
            gate_marker_1.color.g = 0
            gate_marker_1.color.b = 1
            gate_marker_1.color.a = 1.0
            gate_marker_1.lifetime = rospy.Duration(0)
            marker_array.markers.append(gate_marker_1)

            gate_marker_2 = Marker()
            gate_marker_2.header.frame_id = "vehicle_frame"
            gate_marker_2.header.stamp    = rospy.get_rostime()
            gate_marker_2.ns = "effort"
            gate_marker_2.id = 1
            gate_marker_2.type = 1
            gate_marker_2.action = 0
            gate_marker_2.pose.orientation.w = 1
            gate_marker_2.pose.position.y = data.y*5/2.0
            gate_marker_2.scale.x = .04
            gate_marker_2.scale.y = data.y*5
            gate_marker_2.scale.z = .04
            gate_marker_2.color.r = 1
            gate_marker_2.color.g = 0
            gate_marker_2.color.b = 1
            gate_marker_2.color.a = 1.0
            gate_marker_2.lifetime = rospy.Duration(0)
            marker_array.markers.append(gate_marker_2)

            gate_marker_3 = Marker()
            gate_marker_3.header.frame_id = "vehicle_frame"
            gate_marker_3.header.stamp    = rospy.get_rostime()
            gate_marker_3.ns = "effort"
            gate_marker_3.id = 2
            gate_marker_3.type = 1
            gate_marker_3.action = 0
            gate_marker_3.pose.orientation.w = 1
            gate_marker_3.pose.position.z = (data.z)
            gate_marker_3.scale.x = .04
            gate_marker_3.scale.y = .04
            gate_marker_3.scale.z = (data.z)*2
            gate_marker_3.color.r = 1
            gate_marker_3.color.g = 0
            gate_marker_3.color.b = 1
            gate_marker_3.color.a = 1.0
            gate_marker_3.lifetime = rospy.Duration(0)
            marker_array.markers.append(gate_marker_3)

            gate_marker_4 = Marker()
            gate_marker_4.header.frame_id = "vehicle_frame"
            gate_marker_4.header.stamp    = rospy.get_rostime()
            gate_marker_4.ns = "effort"
            gate_marker_4.id = 3
            gate_marker_4.type = 1
            gate_marker_4.action = 0
            gate_marker_4.pose.orientation.w = 1
            gate_marker_4.pose.position.x = .5
            gate_marker_4.pose.position.y = (data.r)/2
            gate_marker_4.scale.x = .04
            gate_marker_4.scale.y = (data.r)
            gate_marker_4.scale.z = .04
            gate_marker_4.color.r = .5
            gate_marker_4.color.g = .5
            gate_marker_4.color.b = .5
            gate_marker_4.color.a = 1.0
            gate_marker_4.lifetime = rospy.Duration(0)
            marker_array.markers.append(gate_marker_4)
            
            self.vichile_pub.publish(marker_array)


        elif args == 'Image':
            pass

        elif args == 'wp_visual':

            if data.pos.z == 0 and data.hdg == 0:
                return


            marker_array = gate_marker("gate_frame_visual")

            self.gate_pub.publish(marker_array)
            
            # self.tbr.sendTransform((data.t[0],data.t[1],data.t[2]),(quat[0],quat[1],quat[2],quat[3]),rospy.get_rostime(),'gate_frame',"odom")
            quat = tf.transformations.quaternion_from_euler(0, 0, data.hdg)
            self.tbr.sendTransform((data.pos.x,data.pos.y,data.pos.z),(quat[0],quat[1],quat[2],quat[3]),rospy.get_rostime(),'gate_frame_visual',"track_frame")



        elif args == 'wp_blind':

            if data.pos.z == 0 and data.hdg == 0:
                return
            
            marker_array = MarkerArray()

            
            quat = tf.transformations.quaternion_from_euler(0, 0, data.hdg)
            
            gate_marker_1 = Marker()
            gate_marker_1.header.frame_id = "gate_frame_blind"
            gate_marker_1.header.stamp    = rospy.get_rostime()
            gate_marker_1.ns = "wp_blind_1"
            gate_marker_1.id = self.gate_number
            gate_marker_1.type = 2
            gate_marker_1.action = 0
            gate_marker_1.pose.orientation.w = 1
            gate_marker_1.scale.x = .25
            gate_marker_1.scale.y = .25
            gate_marker_1.scale.z = .25
            gate_marker_1.color.r = .5
            gate_marker_1.color.g = 0
            gate_marker_1.color.b = 1
            gate_marker_1.color.a = 1.0
            gate_marker_1.lifetime = rospy.Duration(0)
            marker_array.markers.append(gate_marker_1)


            gate_marker_4 = Marker()
            gate_marker_4.header.frame_id = "gate_frame_blind"
            gate_marker_4.header.stamp    = rospy.get_rostime()
            gate_marker_4.ns = "wp_blind_2"
            gate_marker_4.id = self.gate_number
            gate_marker_4.type = 1
            gate_marker_4.action = 0
            gate_marker_4.pose.position.x = -.5
            # gate_marker_4.pose.position.y = 0
            gate_marker_4.pose.orientation.w = 1
            gate_marker_4.scale.x = 1
            gate_marker_4.scale.y = .03
            gate_marker_4.scale.z = .03
            gate_marker_4.color.r = 1
            gate_marker_4.color.g = 0
            gate_marker_4.color.b = 1
            gate_marker_4.color.a = 1.0
            gate_marker_4.lifetime = rospy.Duration(0)
            #marker_array.markers.append(gate_marker_4)
            
            # print 'sending blind wp'
            
            self.gate_pub.publish(marker_array)
            
            self.tbr.sendTransform((data.pos.x,data.pos.y,data.pos.z),(quat[0],quat[1],quat[2],quat[3]),rospy.get_rostime(),'gate_frame_blind',"track_frame")


        elif args == 'wp_look':

            if data.pos.x == 0 and data.pos.y == 0:
                return
            
            marker_array = MarkerArray()
            
            gate_marker_1 = Marker()
            gate_marker_1.header.frame_id = 'wp_look_frame'
            gate_marker_1.header.stamp    = rospy.get_rostime()
            gate_marker_1.ns = "wp_look"
            # gate_marker_1.id = self.gate_number
            gate_marker_1.id = 1
            gate_marker_1.type = 2
            gate_marker_1.action = 0
            gate_marker_1.pose.orientation.w = 1
            gate_marker_1.scale.x = .25
            gate_marker_1.scale.y = .25
            gate_marker_1.scale.z = .75
            gate_marker_1.color.r = 0
            gate_marker_1.color.g = 0
            gate_marker_1.color.b = 1
            gate_marker_1.color.a = 1.0
            gate_marker_1.lifetime = rospy.Duration(0)
            marker_array.markers.append(gate_marker_1)
            
            self.gate_pub.publish(marker_array)            
            self.tbr.sendTransform((data.pos.x,data.pos.y,2),(0,0,0,1),rospy.get_rostime(),'wp_look_frame',"track_frame")

        elif args == 'wp_current':

            if data.pos.x == 0 and data.pos.y == 0:
                return
            
            marker_array = MarkerArray()
            
            gate_marker_1 = Marker()
            gate_marker_1.header.frame_id = 'wp_current_frame'
            gate_marker_1.header.stamp    = rospy.get_rostime()
            gate_marker_1.ns = "wp_current_1"
            gate_marker_1.id = 1
            gate_marker_1.type = 1
            gate_marker_1.action = 0
            gate_marker_1.pose.orientation.w = 1
            gate_marker_1.scale.x = .05
            gate_marker_1.scale.y = .4
            gate_marker_1.scale.z = .4
            gate_marker_1.color.r = 1
            gate_marker_1.color.g = 0
            gate_marker_1.color.b = 0
            gate_marker_1.color.a = 1.0
            gate_marker_1.lifetime = rospy.Duration(0)
            marker_array.markers.append(gate_marker_1)

            gate_marker_4 = Marker()
            gate_marker_4.header.frame_id = "wp_current_frame"
            gate_marker_4.header.stamp    = rospy.get_rostime()
            gate_marker_4.ns = "wp_current_2"
            gate_marker_4.id = 1
            gate_marker_4.type = 1
            gate_marker_4.action = 0
            gate_marker_4.pose.orientation.w = 1
            gate_marker_4.scale.x = 2
            gate_marker_4.scale.y = .03
            gate_marker_4.scale.z = .03
            gate_marker_4.color.r = 1
            gate_marker_4.color.g = .1
            gate_marker_4.color.b = .1
            gate_marker_4.color.a = 1.0
            gate_marker_4.lifetime = rospy.Duration(0)
            marker_array.markers.append(gate_marker_4)
            
            self.gate_pub.publish(marker_array)  

            quat = tf.transformations.quaternion_from_euler(0, 0, data.hdg)         
            self.tbr.sendTransform((data.pos.x,data.pos.y,data.pos.z),(quat[0],quat[1],quat[2],quat[3]),rospy.get_rostime(),'wp_current_frame',"track_frame")

        elif args == 'gate_pose':

            if data.position.x == 0:
                return
            
            pos = data.position
            quat = data.orientation


            marker_array = MarkerArray()
            
            gate_marker_1 = Marker()
            gate_marker_1.header.frame_id = 'gate_frame'
            gate_marker_1.header.stamp    = rospy.get_rostime()
            gate_marker_1.ns = "wp_proj"
            gate_marker_1.id = 0
            gate_marker_1.type = 1
            gate_marker_1.action = 0
            gate_marker_1.pose.orientation.w = 1
            gate_marker_1.scale.x = .05
            gate_marker_1.scale.y = .4
            gate_marker_1.scale.z = .4
            gate_marker_1.color.r = 1
            gate_marker_1.color.g = 0
            gate_marker_1.color.b = 0
            gate_marker_1.color.a = 1.0
            gate_marker_1.lifetime = rospy.Duration(0)
            marker_array.markers.append(gate_marker_1)

            gate_marker_4 = Marker()
            gate_marker_4.header.frame_id = "gate_frame"
            gate_marker_4.header.stamp    = rospy.get_rostime()
            gate_marker_4.ns = "wp_proj"
            gate_marker_4.id = 1
            gate_marker_4.type = 1
            gate_marker_4.action = 0
            gate_marker_4.pose.orientation.w = 1
            gate_marker_4.scale.x = 2
            gate_marker_4.scale.y = .03
            gate_marker_4.scale.z = .03
            gate_marker_4.color.r = 1
            gate_marker_4.color.g = .1
            gate_marker_4.color.b = .1
            gate_marker_4.color.a = 1.0
            gate_marker_4.lifetime = rospy.Duration(0)
            marker_array.markers.append(gate_marker_4)
            
            self.gate_pub.publish(marker_array)  

            # quat = tf.transformations.quaternion_from_euler(0, 0, data.hdg)         
            # self.tbr.sendTransform((data.pos.x,data.pos.y,data.pos.z),(quat[0],quat[1],quat[2],quat[3]),rospy.get_rostime(),'gate_frame',"track_frame")
            # self.tbr.sendTransform((pos.x,pos.y,pos.z),(quat.x,quat.y,quat.z,quat.w),rospy.get_rostime(),'gate_frame',"track_frame")
            self.tbr.sendTransform((pos.x,pos.y,pos.z),(quat.x,quat.y,quat.z,quat.w),rospy.get_rostime(),'gate_frame',"odom")

def main():
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('bebop_rviz', anonymous=True)

    
    
    bebop_data()

    rospy.spin() #root.mainloop()


if __name__ == '__main__':
    main()
