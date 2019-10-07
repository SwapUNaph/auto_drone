#!/usr/bin/env python

# Script developed by Vincenz Frenzel (navigation by Derek Thompson)
#  --- Changelog ---
# Goal:     Main logic of autonomy
# Status:   07/11: Started with script
#           11/02: Added last comments to script

import rospy
import signal
import sys
import math
import numpy as np
import time
from std_msgs.msg import Int32, String, Float32MultiArray, Bool, Float32, Empty
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from auto_drone.msg import Detection_Active, WP_Msg, Drone_Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Quaternion, Point
from tf import transformations as tfs
import common_resources as cr


def signal_handler(_, __):
    # enable Ctrl+C of the script
    sys.exit(0)


def callback_states_changed(data, args):
    # update states, either autonomy state or bebop state
    if args == "state_auto":
        global state_auto
        state_auto = data.data
        rospy.loginfo("state auto changed to " + str(state_auto))
    elif args == "state_bebop":
        global state_bebop
        state_bebop = data.state
        rospy.loginfo("state bebop changed to " + str(state_bebop))
    elif args == 'autonomy':
        global autonomy_active
        autonomy_active = data.data
        rospy.loginfo("autonomy active changed to " + str(autonomy_active))



# no idea what todo here
def callback_visual_gate_detection_changed(data):
    global wp_average
    global wp_input_history
    
    gate_position, gate_hdg = cr.WP2array(data)
    
    
    
    # detection must be active
    if not detection_active:
        rospy.loginfo("detection not active")
        return

    # content must not be empty
    if data.tvec == ():
        rospy.loginfo("empty visual input")
        return
    
    '''
    # read data
    bebop_position = data.bebop_pose.position
    bebop_orientation = data.bebop_pose.orientation
    
    # bebop position and orientation
    bebop_p = np.array([bebop_position.x, bebop_position.y, bebop_position.z])
    bebop_q = np.array([bebop_orientation.x, bebop_orientation.y, bebop_orientation.z, bebop_orientation.w])

    # gate position and orientation
    # rospy.loginfo("tvec")
    # rospy.loginfo(data.tvec)
    # rospy.loginfo("rvec")
    # rospy.loginfo(data.rvec)

    gate_pos = np.array(data.tvec)
    gate_q = cr.axang2quat(np.array(data.rvec))
    #
    # rospy.loginfo("bebop_p")
    # rospy.loginfo(bebop_p)
    # rospy.loginfo("bebop_q")
    # rospy.loginfo(bebop_q)
    # rospy.loginfo("gate_pos")
    # rospy.loginfo(gate_pos)
    # rospy.loginfo("gate_q")
    # rospy.loginfo(gate_q)

    # transform measurement into global frame, calculate headings
    # headings are required to make sure gate is not backwards (it is 1*pi periodic so could be wrong way round)
    gate_global_p = cr.qv_mult(bebop_q, cr.qv_mult(cr.cam_q, gate_pos) + cr.BZ) + bebop_p
    gate_global_q = tfs.quaternion_multiply(bebop_q, tfs.quaternion_multiply(cr.cam_q, gate_q))
    gate_normal_vec = cr.qv_mult(gate_global_q, [0, 0, 1])
    heading_to_gate = math.atan2((gate_global_p[1] - bebop_p[1]), gate_global_p[0] - bebop_p[0])
    heading_of_gate = math.atan2(gate_normal_vec[1], gate_normal_vec[0])
    heading_difference = math.fabs(heading_to_gate - heading_of_gate) * 180 / math.pi
    
    # rospy.loginfo("gate_global_p")
    # rospy.loginfo(gate_global_p)
    # rospy.loginfo("gate_global_q")
    # rospy.loginfo(gate_global_q)
    # rospy.loginfo("gate_normal_vec")
    # rospy.loginfo(gate_normal_vec)

    if 90 < heading_difference < 270:
        if heading_of_gate < 0:
            heading_of_gate = heading_of_gate + math.pi
        else:
            heading_of_gate = heading_of_gate - math.pi

    # accept info as current WP. publish for ground control station
    wp_current = cr.WP(gate_global_p, heading_of_gate)
    msg = WP_Msg()
    msg.pos.x = wp_current.pos[0]
    msg.pos.y = wp_current.pos[1]
    msg.pos.z = wp_current.pos[2]
    msg.hdg = wp_current.hdg
    publisher_wp_current.publish(msg)

    rospy.loginfo("new measured wp_current")
    rospy.loginfo(wp_current)

    # while there is no average yet: collect gate positions
    if wp_average is None:
        wp_input_history.append(wp_current)
        # once enough measurements are there, delete oldest ones
        if len(wp_input_history) > 15:
            rospy.loginfo("enough measurements")
            del wp_input_history[0]
            # calculate average of measurements (average position and angle)
            average = cr.find_average(wp_input_history)
            # calculate std deviation of positions
            std_deviation = cr.find_std_dev_waypoints(average, wp_input_history)
            # when std dev is low enough, provide waypoint ("lock on" to average gate)
            if std_deviation < 0.25:
                rospy.loginfo("measurements accepted")
                wp_average = average
            else:
                rospy.loginfo("standard deviation too high:")
                rospy.loginfo(std_deviation)
        else:
            # not enough measurements yet
            std_deviation = 5
            average = cr.WP([0, 0, 0], 0)
            rospy.loginfo("collecting measurements")

        # logging for matlab
        log_string = str(wp_current.pos[0]) + ", " + \
            str(wp_current.pos[1]) + ", " + \
            str(wp_current.pos[2]) + ", " + \
            str(wp_current.hdg) + ", " + \
            str(average.pos[0]) + ", " + \
            str(average.pos[1]) + ", " + \
            str(average.pos[2]) + ", " + \
            str(average.hdg) + ", " + \
            str(bebop_p[0]) + ", " + \
            str(bebop_p[1]) + ", " + \
            str(bebop_p[2]) + ", " + \
            str(bebop_q[3]) + ", " + \
            str(bebop_q[0]) + ", " + \
            str(bebop_q[1]) + ", " + \
            str(bebop_q[2]) + ", " + \
            str(heading_to_gate) + ", " + \
            str(std_deviation) + ", " + \
            str(1.0) + ", " + \
            str(time.time()-t_log)
    else:
        # now, average is established
        # add measurement to list only if gate position and hdg is close to average
        distance = np.linalg.norm(wp_current.pos - wp_average.pos)
        hdg_diff = abs(math.atan2(math.sin(wp_current.hdg - wp_average.hdg), math.cos(wp_current.hdg - wp_average.hdg)))
        if distance < 0.4 and hdg_diff < 15*math.pi/180:
            # measurements are accepted. update history and recalculate average
            rospy.loginfo("use detected gate")
            wp_input_history.append(wp_current)
            del wp_input_history[0]
            wp_average = cr.find_average(wp_input_history)
            rospy.loginfo("wp_average")
            rospy.loginfo(wp_average)
        else:
            # measurement rejected
            rospy.loginfo("discard detected gate")

        # logging for matlab
        log_string = str(wp_current.pos[0]) + ", " + \
            str(wp_current.pos[1]) + ", " + \
            str(wp_current.pos[2]) + ", " + \
            str(wp_current.hdg) + ", " + \
            str(wp_average.pos[0]) + ", " + \
            str(wp_average.pos[1]) + ", " + \
            str(wp_average.pos[2]) + ", " + \
            str(wp_average.hdg) + ", " + \
            str(bebop_p[0]) + ", " + \
            str(bebop_p[1]) + ", " + \
            str(bebop_p[2]) + ", " + \
            str(bebop_q[3]) + ", " + \
            str(bebop_q[0]) + ", " + \
            str(bebop_q[1]) + ", " + \
            str(bebop_q[2]) + ", " + \
            str(heading_to_gate) + ", " + \
            str(distance) + ", " + \
            str(2.0) + ", " + \
            str(time.time()-t_log)

    publisher_visual_log.publish(log_string)

    # WP for ground control station
    msg = WP_Msg()
    if wp_average is not None:
        msg.pos.x = wp_average.pos[0]
        msg.pos.y = wp_average.pos[1]
        msg.pos.z = wp_average.pos[2]
        msg.hdg = wp_average.hdg
    publisher_wp_average.publish(msg)

    rospy.loginfo("visual gate done")

    '''


# needs work
def navigate_through():
    # navigation algorithm to carry us onto the centerline and up to the gate, all navigation written by Derek
    bebop_position = bebop_odometry.pose.pose.position
    bebop_orientation = bebop_odometry.pose.pose.orientation

    bebop_p = [bebop_position.x, bebop_position.y, bebop_position.z]
    bebop_q = [bebop_orientation.x, bebop_orientation.y, bebop_orientation.z, bebop_orientation.w]
    bebop_x_vec = cr.qv_mult(bebop_q, [1, 0, 0])
    hdg = math.atan2(bebop_x_vec[1], bebop_x_vec[0])

    rospy.loginfo("fly from")
    rospy.loginfo([bebop_p[0], bebop_p[1], bebop_p[2], hdg])
    rospy.loginfo("fly to")
    rospy.loginfo(current_state.fly.pos)

    # heading angle
    angle = tfs.euler_from_quaternion(bebop_q)[2]

    # bebop velocity
    velocity = bebop_odometry.twist.twist.linear

    # global difference between WP ad position
    diff_global = current_state.fly.pos - bebop_p
    # X and Y components hypothenuse
    dist = math.hypot(diff_global[0], diff_global[1])

    # heading of the gate itself
    gate_theta = current_state.fly.hdg
    # heading from drone to gate position
    pos_theta = math.atan2(diff_global[1], diff_global[0])

    # difference between the two headings
    d_theta = gate_theta - pos_theta
    if d_theta > math.pi:
        d_theta = -2 * math.pi + d_theta
    elif d_theta < -math.pi:
        d_theta = 2 * math.pi + d_theta
    else:
        pass

    # lateral deviation from centerline
    y_pos_error = -dist * math.sin(d_theta)
    y_vel_des = nav_through_PID_y_pos.update(y_pos_error)

    # axial deviation to gate
    x_pos_error = cr.min_value(dist * math.cos(d_theta), 0.1)
    if dist > 2:
        # if further than 2m: be within 30 degrees to get x velocity desired
        x_vel_des = x_pos_error*max(cr.limit_value(1-6*abs(d_theta)/math.pi, 1.0), 0)
    else:
        # if closer than 2m: be within 13 degrees to get x velocity desired
        x_vel_des = x_pos_error*max(cr.limit_value(1-14*abs(d_theta)/math.pi, 1.0), -.25)

    # height difference
    z_error = diff_global[2]

    # yaw difference
    r_error = -(angle - pos_theta)
    if r_error > math.pi:
        r_error = -2 * math.pi + r_error
    elif r_error < -math.pi:
        r_error = 2 * math.pi + r_error

    # special algorithm that limits lateral velocity. up to 0.1 use proportional. everything above: squeeze to a third
    y_vel_des_sum = sum(y_vel_des)
    if y_vel_des_sum > 0.1:
        y_vel_des_sum = (y_vel_des_sum - 0.1)/3 + 0.1
    elif y_vel_des_sum < -0.1:
        y_vel_des_sum = (y_vel_des_sum + 0.1)/3 - 0.1

    # velocity error
    y_vel_error = y_vel_des_sum - velocity.y
    # don't allow too high x velocity
    x_vel_error = cr.limit_value(x_vel_des, 0.6) - velocity.x

    # update PID loops
    nav_cmd_x = nav_through_PID_x_vel.update(x_vel_error)
    nav_cmd_y = nav_through_PID_y_vel.update(y_vel_error)
    nav_cmd_z = nav_through_PID_z_vel.update(z_error)
    nav_cmd_r = nav_through_PID_r_vel.update(r_error)

    # create message. limit with global limits, add a part to x to always move forward a bit
    msg = Twist()
    msg.linear.x = cr.limit_value(sum(nav_cmd_x) + 0.04, nav_limit_x_thr)
    msg.linear.y = cr.limit_value(sum(nav_cmd_y), nav_limit_y_thr)
    msg.linear.z = cr.limit_value(sum(nav_cmd_z), nav_limit_z)
    msg.angular.r = cr.limit_value(sum(nav_cmd_r), nav_limit_r)

    bebop_model.update_att([msg.x,msg.y,msg.z,msg.r])


    log_array = Float32MultiArray()

    log_array.data = [
        x_vel_des,
        velocity.x,
        x_vel_error,
        nav_cmd_x[0],
        nav_cmd_x[1],
        nav_cmd_x[2],
        sum(nav_cmd_x),
        msg.linear.x,
        y_pos_error,
        y_vel_des[0],
        y_vel_des[1],
        y_vel_des[2],
        sum(y_vel_des),
        velocity.y,
        y_vel_error,
        nav_cmd_y[0],
        nav_cmd_y[1],
        nav_cmd_y[2],
        sum(nav_cmd_y),
        msg.linear.y,
        diff_global[2],
        z_error,
        nav_cmd_z[0],
        nav_cmd_z[1],
        nav_cmd_z[2],
        sum(nav_cmd_z),
        msg.linear.z,
        pos_theta,
        angle,
        r_error,
        nav_cmd_r[0],
        nav_cmd_r[1],
        nav_cmd_r[2],
        sum(nav_cmd_r),
        msg.angular.r]

    log_array.layout.dim = len(log_array.data)


    publisher_nav_log.publish(log_array)
    # command itself is returned
    return msg



# needs fix to pose get
def navigate_point():
	
    # navigation algorithm to fly us to WPs    
    bebop_p = bebop_model.pos

    hdg = bebop_model.att[2]
    

    target = current_state.fly.pos
    target_look = current_state.look.pos
    # transform velocities into global frame
    global_vel = bebop_model.vel

    
    diff_global = current_state.fly.pos - bebop_p
    

    rospy.loginfo("fly from")
    rospy.loginfo([bebop_p[0], bebop_p[1], bebop_p[2], hdg])
    rospy.loginfo("fly to")
    rospy.loginfo(current_state.fly.pos)
    
    
    # LATERAL CONTROLLER
    
    x_pos_error = diff_global[0]
    y_pos_error = diff_global[1]

    x_pos_proj = lin_pos.x + global_vel[0] * point_time_proj
    y_pos_proj = lin_pos.y + global_vel[1] * point_time_proj

    
    x_vel_des_proj = min(max(point_vel_slope*(target[0]-x_pos_proj),-point_nav_des_vel),point_nav_des_vel)
    y_vel_des_proj = min(max(point_vel_slope*(target[1]-y_pos_proj),-point_nav_des_vel),point_nav_des_vel)

    
    x_vel_error = x_vel_des_proj - global_vel[0]
    y_vel_error = y_vel_des_proj - global_vel[1]


    # min_arrive_time = .1
    # x_arrive_time = max(diff_global[0]/global_vel[0],min_arrive_time)
    # y_arrive_time = max(diff_global[1]/global_vel[1],min_arrive_time)
    x_arrive_time = point_time_proj
    y_arrive_time = point_time_proj

    x_accel_time = min(point_time_proj,x_arrive_time)
    y_accel_time = min(point_time_proj,y_arrive_time)
    
    #x_accel_des = x_vel_error/(x_accel_time) + x_vel_des * nav_drag    
    #y_accel_des = y_vel_error/(y_accel_time) + y_vel_des * nav_drag    
    x_accel_des = x_vel_error/(x_accel_time) + x_vel_des_proj * nav_drag    
    y_accel_des = y_vel_error/(y_accel_time) + y_vel_des_proj * nav_drag
    
    x_theta_des = np.arctan(x_accel_des/9.81)
    y_theta_des = np.arctan(y_accel_des/9.81)

    nav_cmd_x = (x_theta_des * 180 / np.pi) / 40
    nav_cmd_y = (y_theta_des * 180 / np.pi) / 40


    # limit commands
    nav_cmd_x_lim = min(max(nav_cmd_x, -point_nav_limit_lat),point_nav_limit_lat)
    nav_cmd_y_lim = min(max(nav_cmd_y, -point_nav_limit_lat),point_nav_limit_lat)

    
    # return navigation commands back into vehicle frame
    nav_cmd_x_veh = nav_cmd_x_lim * np.cos(-hdg) - nav_cmd_y_lim * np.sin(-hdg)
    nav_cmd_y_veh = nav_cmd_y_lim * np.cos(-hdg) + nav_cmd_x_lim * np.sin(-hdg)
    


    # Z CONTROLLER

    z_error = diff_global[2]
    nav_cmd_z = nav_PID_z.update(z_error)
    


    # R CONTROLLER
    
    diff_global_look = target_look - bebop_p
    pos_theta = np.arctan2(diff_global_look[1], diff_global_look[0])

    r_error = -(hdg - pos_theta)
    if r_error > np.pi:
        r_error = -2 * np.pi + r_error
    elif r_error < -np.pi:
        r_error = 2 * np.pi + r_error

    nav_cmd_r = nav_PID_r.update(z_error)
    


    
    # Create twist msg
    msg = Twist()
    msg.linear.x = point_lateral_gain*nav_cmd_x_veh
    msg.linear.y = point_lateral_gain*nav_cmd_y_veh
    msg.linear.z = cr.limit_value(sum(nav_cmd_z), nav_limit_z)
    msg.angular.z = cr.limit_value(sum(nav_cmd_r), nav_limit_r)



    log_array = Float32MultiArray()

    log_array.data = [x_pos_error,
        x_pos_proj,
        x_vel_des_proj,
        global_vel[0],
        x_vel_error,
        x_accel_time,
        x_accel_des,
        x_theta_des,
        nav_cmd_x,
        nav_cmd_x_lim,
        msg.linear.x,
        y_pos_error,
        y_pos_proj,
        y_vel_des_proj,
        global_vel[1],
        y_vel_error,
        y_accel_time,
        y_accel_des,
        y_theta_des,
        nav_cmd_y,
        nav_cmd_y_lim,
        msg.linear.y,
        z_error,
        nav_cmd_z[0],
        nav_cmd_z[1],
        nav_cmd_z[2],
        sum(nav_cmd_z),
        msg.linear.z,
        pos_theta,
        angle,
        r_error,
        nav_cmd_r[0],
        nav_cmd_r[1],
        nav_cmd_r[2],
        sum(nav_cmd_r),
        msg.angular.z]


    log_array.layout.dim.append(MultiArrayDimension())

    log_array.layout.dim[0].label = 'point_logs'
    log_array.layout.dim[0].size = len(log_array.data)

    self.nav_log_pub.publish(log_array)


    publisher_nav_log.publish(log_array)
    return msg


# need to fix distance through nav
def calculate_distance():
    # calculate a distance based on which navigation is active

    global bebop_model
    
    # no waypoint or no odometry
    if current_state.fly is None or bebop_model.pos is None:
        return 999

    diff_global = current_state.fly.pos - bebop_model.pos

    if nav_active == "point":
        # use linear 3D distance between two points. Add penalty for angular offset
        linear_distance = np.linalg.norm(diff_global)

        return linear_distance 


    elif nav_active == "through":
        # in any through navigation, use a plane instead that is parallel to gate itself. Eventually it doesn't matter
        # how far we are from the gate in 3D but how far we are from the plane of the gate
        flat_distance = np.linalg.norm([diff_global[0], diff_global[1], 0])
        heading_to_gate = math.atan2(current_state.fly.pos[1] - bebop_position.y, current_state.fly.pos[0] - bebop_position.x)
        heading_of_gate = current_state.fly.hdg
        heading_difference = heading_to_gate - heading_of_gate
        return flat_distance * math.cos(heading_difference)

        


class State:
    # This is the ominous state machine
    def __init__(self, own_state=None, next_state=None, condition_type=None, condition_thres=None,
                 exit_clear_visual=None, detection_active_bool=None, nav_active_str=None,
                 gate_color=None, fly=None, look=None,gate_orientation=None):
        self.own_state = own_state                              # own state id
        self.next_state = next_state                            # next state id
        self.condition_type = condition_type                    # type of state advancement
        self.condition_thres = condition_thres                  # value for state advancement (not req. for all)
        self.exit_clear_visual = bool(exit_clear_visual)        # on state exit, clear all visual data? (new gate)
        self.detection_active = bool(detection_active_bool)     # detection active?
        self.nav_active = nav_active_str                        # which navigation should be active
        self.gate_color = gate_color                            # color of the gate
        self.fly = fly                                          # if blind waypoint should be calculated: offset for
        self.look = look                                        # flying and looking waypoint into gate direction
        self.time = None                                        # timer for takeoff wait
        self.gate_orientation = gate_orientation                # If it needs to guess the gate
		
    def enter(self):
        # do things when state is selected
        print("enter state " + str(self.own_state))
        # change global state id and also publish state id
        global current_state
        current_state = states[self.own_state]
        publisher_state_auto.publish(self.own_state)

        # reset all PID loops
        nav_PID_z.reset()
        nav_PID_r.reset()
        # na
        # nav_through_PID_y_pos.reset()
        # nav_through_PID_x_vel.reset()
        # nav_through_PID_y_vel.reset()
        # nav_through_PID_z_vel.reset()
        # nav_through_PID_r_vel.reset()

        # if timer is required for condition, start timer
        if self.condition_type == "time":
            self.time = time.time()


        # if gate color is defined, publish gate color
        if self.gate_color is not None:
            msg = Float32MultiArray()
            msg.data = self.gate_color
            publisher_gate_color.publish(msg)
        
        # set applicable navigation
        global nav_active
        nav_active = self.nav_active

        # set detection to on/off
        global detection_active
        detection_active = self.detection_active

        # # calculate blind waypoints
        # if self.fly.any():
        #     rospy.loginfo("state " + str(self.own_state) + ": set blind waypoint")
        #     calculate_blind_waypoint(self.fly, self.look)


        if self.gate_orientation is not None:
            if self.gate_orientation == 'vertical':
                pass

            elif gate_orientation == 'horizontal':
                pass


        # publish blind WP for ground control station
        if self.look is not None:
            msg = WP_Msg()    
            msg.pos.x = self.look[0]
            msg.pos.y = self.look[1]
            msg.pos.z = self.look[2]
            msg.hdg = 0
            publisher_wp_look.publish(msg)

        
        # publish look WP for ground control station
        if self.fly is not None:
            msg = WP_Msg()
            
            if self.nav_active == 'through':
                msg.pos.x = self.fly.pos[0]
                msg.pos.y = self.fly.pos[1]
                msg.pos.z = self.fly.pos[2]
                if self.fly.hdg is not None:
                    msg.hdg = self.fly.hdg
                publisher_wp_blind.publish(msg)



        
    def exit(self):
        # do things when state is finished
        print("exit state " + str(self.own_state))

        # if visuals should be cleared, do it now (preparation for next gate)
        if self.exit_clear_visual:
            global wp_average
            global wp_input_history
            wp_average = None
            wp_input_history = []

        # enter new state
        states[self.next_state].enter()


    def check(self, navigation_distance):
        # setup state advancement condition
        rospy.loginfo("state " + str(state_auto))

        # distance: if distance is smaller than value (could be scaled if there is a scale)
        if self.condition_type == "dist":
            if navigation_distance < self.condition_thres:
                self.exit()

        # waypoint: if average waypoint is found (not None anymore). visual_wp is "locked on"
        elif self.condition_type == "wp":
            if gate_average is not None:
                self.exit()

        # specific condition of bebop state machine (hover, land, ...)
        elif self.condition_type == "bebop":
            if state_bebop == self.condition_thres:
                self.exit()

        # some time has elapsed (on takeoff). Required because magnetometer changes a lot while motors spin up and drone
        # takes off
        elif self.condition_type == "time":
            if time.time() > self.time + self.condition_thres:
                self.exit()



def callback_bebop_odometry_changed(data):
    # main loop, every time odometry changes
    # global bebop_odometry
    bebop_pose = data.pose.pose.position
    quat = data.pose.pose.orientation
    bebop_hdg = tfs.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]
    rospy.loginfo("Odom recieved: " + str([bebop_pose.x, bebop_pose.y, bebop_pose.z, bebop_hdg]))


    global bebop_model
    global nav_active
    global bebop_last_known_hdg
    global bebop_last_known_pos
    global bebop_odometry
    



    # if there is no odometry, don't do anything
    if bebop_odometry is None:
        rospy.loginfo("No position")
        rospy.loginfo("publish empty driving msg")
        publisher_auto_drive.publish(Twist())
        bebop_odometry = data
        return

    if current_state is None:
        rospy.loginfo("No State")
        publisher_auto_drive.publish(Twist())
        bebop_odometry = data
        return        

    
    # if there is no track frame reference point, set current position to takeoff 
    if bebop_last_known_pos is None:
        update_last_known_bebop(data)


    # if point nav or nav off update pose to track frame and update
    if nav_active == "off" or  nav_active == "point":
        bebop_odom_transformed = bebop2track_transform_odom(data)
        bebop_model.update_odom(bebop_odom_transformed)

    elif nav_active == 'through':
        pass


    # Send out bebops estimated position
    publisher_pose.publish(bebop_model.pose)
    

def update_last_known_bebop(odom):

    pos = odom.pose.pose.position
    quat = odom.pose.pose.orientation

    bebop_last_known_pos = np.array([pos.x, pos.y, pos.z])
    bebop_last_known_hdg = tfs.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]


def bebop2track_transform_odom(bebop_odom):
    pose = bebop_odom.pose.pose
    twist = bebop_odom.twist.twist


    bebop_d_pos = np.array([pose.position.x, pose.position.y, pose.position.z]) - bebop_last_known_pos
    bebop_q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

    track_bebop_dpos = track_last_known_pos - bebop_last_known_pos
    track_bebop_dhdg = track_last_known_hdg - bebop_last_known_hdg
    
    R_hdg = tfs.rotation_matrix(track_bebop_dhdg,(0,0,1))[0:3,0:3]



    track_pos = (np.matmul(bebop_d_pos,R_hdg)) + track_last_known_pos
        
    track_odom = bebop_odom
    
    track_odom.pose.pose.position.x = track_pos[0]
    track_odom.pose.pose.position.y = track_pos[1]
    track_odom.pose.pose.position.z = track_pos[2]

    bebop_hdg_LK = tfs.quaternion_multiply(bebop_q,cr.axang2quat([0,0,track_bebop_dhdg]))
    track_hdg = tfs.quaternion_multiply(bebop_hdg_LK,cr.axang2quat([0,0,track_last_known_hdg]))

    track_odom.pose.pose.orientation.x = track_hdg[0]
    track_odom.pose.pose.orientation.y = track_hdg[1]
    track_odom.pose.pose.orientation.z = track_hdg[2]
    track_odom.pose.pose.orientation.w = track_hdg[3]
    return track_odom


def update_pose_estimate():
    #global nav_active
    #global bebop_model
    
    rospy.loginfo("Updating and sending Pose")
    
    # if nav_active == 'point' or  nav_active == 'off':
    #     if bebop_last_known_pos is not None:
    bebop_model.propagate(.05)
	
    publisher_model.publish(bebop_model.get_drone_pose())
    
	# calculate distance from WP
    navigation_distance = calculate_distance()
    rospy.loginfo("navigation distance")
    rospy.loginfo(navigation_distance)

    # state_machine_advancement (if conditions are met: distances, states, ...)
    current_state.check(navigation_distance)


def update_controller():
    
    if nav_active == "off":
        # navigation off, send empty message and return
        rospy.loginfo("Navigation turned off")
        #rospy.loginfo("publish empty driving msg")
        publisher_auto_drive.publish(Twist())
        return

    elif nav_active == "point":
        if bebop_last_known_pos is not None:
            auto_driving_msg = navigate_point()
            publisher_auto_drive.publish(auto_driving_msg)
    
    elif nav_active == "through":
        auto_driving_msg = navigate_through()
        publisher_auto_drive.publish(auto_driving_msg)
        

def emergency_shutdown(_):
    # if this is triggered, it shuts off node for performance reasons
    rospy.loginfo("emergency shutdown")
    rospy.signal_shutdown("emergency shutdown")


if __name__ == '__main__':
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    # initialize node
    rospy.init_node('main_navigation', anonymous=False)

    # Variables
    autonomy_active = False                                     # autonomous mode is active
    
    start_pos = np.array([.7, 5, 0])
    start_hdg = -3.1415/2
    bebop_model = cr.Bebop_Model(start_pos,start_hdg)            # Initialize odometry message to store position

    bebop_odometry = None                                       # latest odometry from bebop
    bebop_last_known_pos = None                                 # keep track of last known position of bebop for tranform to track frame
    bebop_last_known_hdg = None
    track_last_known_pos = start_pos
    track_last_known_hdg = start_hdg

    state_auto = 0                                             # initialize state machine (number of state)
    current_state = None                                        # the actual state object with method "check"
    state_bebop = None                                          # state of drone itself (own state machine)
    

    gate_filtered = None                                        # Filtered gate input from gate detection
   


    wp_visual = None                                            # almost same as above (see calculate_visual_wp)
    wp_visual_old = None                                        # backup of last visual wp
    wp_input_history = []                                       # history of gate measurements
    wp_blind = None                                             # wp calculated from map layout
    wp_blind_old = None                                         # backup of last blind wp
    
    
    
    detection_active = False                                    # gate detection active boolean
    nav_active = "off"                                          # activated navigation algorithm
    
    
    # Controller Classes
    nav_PID_z = cr.PID(1.0, 0, 0.0)
    nav_PID_r = cr.PID(0.5, 0, 1.0)
    

    # Controller Params
    nav_limit_x_thr = .1  # .25                                 # absolute limit to all cmd before being sent, thr_nav
    nav_limit_y_thr = .2  # .4
    nav_limit_z = .5  # .75
    nav_limit_r = 1.0  # 1

    point_time_proj = 0.7
    point_nav_des_vel = 1.5
    point_vel_slope = 2.0
    point_lateral_gain = 1.0
    point_nav_limit_lat = .3
    point_nav_limit_z = .2
    point_nav_limit_r = .8
    
    nav_drag = 0.0


    dist_gate_blind = 1.0                                       # how exact go to blind wp before advancing state
    dist_gate_close = 0.5                                       # how close to gate before advancing state (approach)
    dist_exit_gate_wp = 20.0                                    # how far away is the exit waypoint
    exit_thrs = dist_exit_gate_wp - .75                         # how far behind the gate is it considered passed
    

    # Publishers
    publisher_state_auto = rospy.Publisher("/auto/state_auto",     Int32,                queue_size=1, latch=True)
    publisher_auto_drive = rospy.Publisher("/auto/auto_drive",     Twist,                queue_size=1, latch=True)
    publisher_model = rospy.Publisher("/auto/pose",                Drone_Pose,          queue_size=1, latch=True)

    publisher_wp_average = rospy.Publisher("/auto/wp_average",     WP_Msg,               queue_size=1, latch=True)
    publisher_wp_visual = rospy.Publisher("/auto/wp_visual",       WP_Msg,               queue_size=1, latch=True)

    publisher_wp_blind = rospy.Publisher("/auto/wp_blind",         WP_Msg,               queue_size=1, latch=True)
    publisher_wp_look = rospy.Publisher("/auto/wp_look",           WP_Msg,               queue_size=1, latch=True)

    publisher_wp_current = rospy.Publisher("/auto/wp_current",     WP_Msg,               queue_size=1, latch=True)
    publisher_nav_log = rospy.Publisher("/auto/navigation_logger", Float32MultiArray,    queue_size=1, latch=True)
    publisher_visual_log = rospy.Publisher("/auto/visual_logger",  Float32MultiArray,    queue_size=1, latch=True)
    publisher_dev_log = rospy.Publisher("/auto/dev_logger",        String,               queue_size=1, latch=True)

    publisher_gate_size = rospy.Publisher("/auto/gate_size",       Float32,              queue_size=1, latch=True)
    publisher_gate_color = rospy.Publisher("/auto/gate_color",     Float32MultiArray,    queue_size=1, latch=True)
	
	
	
    # abbreviations for state machine setup
    p = "point"
    t = "through"
    o = "off"

    # Params if needs to guess gate
    # 0 No gate detection
    v = 'vertical' # Gate detection vertical
    h = 'horizontal' # Gate detection vertical

    # Madrid color values for gates
    gate_params1 = np.array([100, 140, 50, 140, 255, 255])

    
    start_pos = cr.WP(np.array([0.7, 5, 0]),None)

    gate_wp_1 = cr.WP(np.array([-0.7, 0.0, 1.7]),-np.pi/2)
    gate_wp_2 = cr.WP(np.array([1.4, 13, 1.7]),np.pi/2)
    
    look_pos_1 = cr.WP(np.array([-0.7, 7, 1.7]),None)
    look_pos_2 = cr.WP(np.array([1.4, 6, 1.7]),None)

    far_pos_1 = cr.WP(np.array([-0.7, -dist_exit_gate_wp, 1.7]),None)
    far_pos_2 = cr.WP(np.array([0.7, 13+dist_exit_gate_wp, 1.7]),None)

    turn_pos_1 = cr.WP(np.array([2.0, -1.25, 1.7]),None)
    turn_pos_2 = cr.WP(np.array([-1.5, 14.25, 1.7]),None)

    # own_state, next_state, condition_type, condition_thres, exit_clear_visual, detection_active_type, nav_active_str, gate_color, fly, look 
    
    states = [State()] * 100
    # states[02] = State(02, 03, "bebop", cr.Bebop.TAKEOFF,  0, 0, o, None, None, None)                                  # Landed
    # states[03] = State(03, 04, "bebop", cr.Bebop.HOVERING, 0, 0, o, None, None, None)                                  # Taking off
    # states[04] = State(04, 10, "time",  1.0,               0, 0, o, None, None, None)                                  # Hovering

    states[10] = State(10, 90, "dist",  0.4,               0, 0, p, None, look_pos_1, gate_wp_1)				         # testing 


    states[11] = State(11, 12, "wp",    None,              0, v, p, gate_params1, look_pos_1, gate_wp_1)                 # Move and Look for gate
    states[12] = State(12, 13, "dist",  dist_gate_close,   1, v, t, gate_params1, None, None)                            # move through gate until cant see it
    states[13] = State(13, 14, "dist",  exit_thrs,         0, 0, p, None, far_pos_1, turn_pos_1)                         # Move through gate
    states[14] = State(14, 21, "dist",  dist_gate_blind,   0, 0, p, None, turn_pos_1, gate_wp_2)                         # Turn around manuever

    '''
    states[20] = State(20, 21, "dist",  dist_gate_blind,   0, 0, p, None, [1.7, 0, 0], [5.0, 0, 0])               
    states[21] = State(21, 22, "wp",    None,              0, 1, p, gate_params1, [3.25, 0, 0], [5.0, 0, 0])
    states[22] = State(22, 23, "dist",  dist_gate_close,   1, 1, t, gate_params1, None, None)
    states[23] = State(23, 31, "dist",  exit_thrs,    0, 0, p, None, [dist_egw, 0, 0], [dist_egw, 0, 0])

    states[30] = State(30, 31, "dist",  dist_gate_blind,   0, 0, p, None, [1.5, -0.25, 0], [0.7, -2.75, 0])
    states[31] = State(31, 32, "wp",    None,              0, 1, p, gate_params1, [0.9, -0.5, -0.3], [0.7, -2.75, 0])
    states[32] = State(32, 33, "dist",  dist_gate_close,   1, 1, t, gate_params1, None, None)
    states[33] = State(33, 10, "dist",  exit_thrs,    0, 0, p, None, [dist_egw, 0, 0], [dist_egw, 0, 0])
    '''

    states[90] = State(90, 91, "bebop", cr.Bebop.LANDING,  0, 0, o, None, None, None)		# land
    states[91] = State(91, 91, "bebop", cr.Bebop.LANDED,   0, 0, o, None, None, None)       # landed

    # Subscribers
    rospy.Subscriber("/auto/state_auto", Int32, callback_states_changed, "state_auto")
    rospy.Subscriber("/auto/autonomy_active", Bool, callback_states_changed, "autonomy")
    rospy.Subscriber("/bebop/odom", Odometry, callback_bebop_odometry_changed)
    rospy.Subscriber("/auto/filtered_gate_WP", WP_Msg, callback_visual_gate_detection_changed)
    rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged,callback_states_changed, "state_bebop")
    rospy.Subscriber("/auto/emergency_shutdown", Empty, emergency_shutdown)

    # initializes startup by publishing state 0
    publisher_state_auto.publish(0)

    
    
    rospy.loginfo("Waiting for autonomy")    
    while not autonomy_active:
        time.sleep(.25)

    update_last_known_bebop(bebop_odometry)

    rospy.loginfo("Autonomy active, starting states")
    # enter state 10 in state machine
    publisher_state_auto.publish(10)
    print states[10]
    states[10].enter()

        

    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
		rospy.loginfo('Controller Loop')
		update_pose_estimate()
		#update_controller()
		rate.sleep()
        

    # rospy.spin()
