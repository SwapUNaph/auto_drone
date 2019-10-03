#!/usr/bin/python
'''
Program description: The planner takes in the current pose of the drone
					 from pose estimator, plans the trajectory, calculates 
					 the error in pose and sends the pose error to the
					 controller.				 
Author: Swapneel Naphade (snaphade@umd.edu)
version: 2.0
Date: 6/12/19
Change log: 
	7/16/19: Added trajectory class.
	8/22/19: Added B-Spline trajectories.
	8/23/19: Added desired velocity in trjaectory.	
'''
import rospy
import signal
import sys 
from geometry_msgs.msg import Twist, Pose, Quaternion, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Empty, Bool, Header
from auto_drone.msg import Traj_error
import transformations as tfs
from common_resources import *
import numpy as np
# from scipy.interpolate import BSpline
import math
from time import time
drone_pose = Pose()
init_pose = Pose()
init_pose.orientation.w = 1

drone_vel = Twist()

target_pose = Pose()
target_pose.orientation.w = 1 
target_pose.position.z = 0

AUTONOMY = False
GATE_IN_VIEW = True


class Trajectory:
    '''
    Class description: This class stores the desired pose and velocities of
                        the trajectory.
    '''

    def __init__(self, path = [], tolerance = 0.5, cycle = True):
        self.path = path
        self.tolerance = tolerance
        self.prev_current_point = 0
        self.cycle = cycle
        if len(path):
            self.current_point = self.path[0]
        else:
            self.current_point = (Pose(), Twist())
            self.current_point[0].orientation.w = 1
                        
    def update_current_point(self, drone_position):
        min_dist_point_ind = 0
        min_distance = 1e5
        
        # Check for the trajectory point closest to drone
        for ind in range(len(self.path)):
            traj_pos, traj_vel = pose2array(self.path[ind][0])
            distance = np.linalg.norm( drone_position - traj_pos )
            if distance < min_distance:
                min_distance = distance
                min_dist_point_ind = ind
        
        # Check if the nearest point is ahead of the drone,
        # if not then make the next	point ahead, the current point
        if self.prev_current_point <= 2:
            if min_dist_point_ind > self.prev_current_point:
                self.current_point = self.path[min_dist_point_ind]
                self.prev_current_point = min_dist_point_ind
        elif min_dist_point_ind < len(self.path)-1: 
            self.current_point = self.path[min_dist_point_ind + 1]
            self.prev_current_point = min_dist_point_ind + 1
        
        # Cycle the trajectory if cycle=true	
        if self.prev_current_point == len(self.path)-1:
            self.current_point = self.path[0]
            self.prev_current_point = 0
            
                                
    def setPath(self, path):
        self.path = path

# Create trajectory instance
trajectory = Trajectory()



def generatePath(gate_position_wrt_ground):
    '''
    Generate path from the gate coordinates.
    '''

    path = []

    # Sim path
    #path.append(array2pose([0,0,2], np.array([0,0,0,1])))
    #path.append(array2pose(gate_position_wrt_ground + np.array([-1,0,0]), np.array([0,0,0,1]))) # First position in front of the gate
    ##path.append(array2pose(gate_position_wrt_ground + np.array([-1,0,0]), np.array([0,0,0,1]))) 
    #path.append(array2pose(gate_position_wrt_ground + np.array([0.5,0,0]), np.array([0,0,0,1]))) # Gate position
    ##path.append(array2pose(gate_position_wrt_ground + np.array([3,0,0]), np.array([0,0,0,1]))) 
    #path.append(array2pose(gate_position_wrt_ground + np.array([1,1.5,0]), np.array([0,0,0.707,0.707]))) 
    ##path.append(array2pose(gate_position_wrt_ground + np.array([3,3,0]), np.array([0,0,0.707,0.707]))) 
    #path.append(array2pose(gate_position_wrt_ground + np.array([1,2,0]), np.array([0,0,1,0]))) 
    #path.append(array2pose(gate_position_wrt_ground + np.array([-2,2,0]), np.array([0,0,1,0]))) 
    ##path.append(array2pose(gate_position_wrt_ground + np.array([-10,3,0]), np.array([0,0,0.707,-0.707]))) 
    #path.append(array2pose(gate_position_wrt_ground + np.array([-2,2,0]), np.array([0,0,0.707,-0.707]))) 
    #path.append(array2pose(gate_position_wrt_ground + np.array([-2,0,0]), np.array([0,0,0,1]))) 

    # Take-off -> Move -> Land
    #path.append(array2pose([0,0,1], np.array([0,0,0,1])))
    #path.append(array2pose([1,0,1], np.array([0,0,0,1])))
    #path.append(array2pose([1,0,1], np.array([0,0,1,0])))
    #path.append(array2pose([0,0,1], np.array([0,0,1,0])))
    #path.append(array2pose([0,0,1], np.array([0,0,0,1])))
    #path.append(array2pose([0,0,1], np.array([0,0,0,1])))
    #path.append(array2pose([1,0,1], np.array([0,0,1,0])))
    #path.append(array2pose([0,0,1], np.array([0,0,0,1])))
    #path.append(array2pose([1,0,1], np.array([0,0,0,1])))

        
    # Take-off -> Pass_gate -> Land
    #path.append(array2pose([0,0,2], np.array([0,0,0,1])))
    #path.append(array2pose([1,0,2], np.array([0,0,0,1])))

    '''
    # B-Spline path
    #print("gate_pose shape: {}".format(len(gate_position_wrt_ground)))
    gate_position_wrt_ground = np.append(gate_position_wrt_ground, [0])
    print(gate_position_wrt_ground)

    traj_rel_gate = np.array([[-6, 0, 0, 0]]*3 + [[-1, 0, 0, 0]]*2 + [[0, 0, 0, 0]] + [[1.5, 0, 0, 0]]*2 + [[1.5, 2, 0 , math.pi/2]]*2 + [[0, 3, 0, math.pi]] + [[-4, 3, 0, math.pi]] + [[-4, 3, 0, 3*math.pi/2]] + [[-4, 3, 0, 2*math.pi]])
    trajectory = traj_rel_gate + gate_position_wrt_ground

    x=trajectory[:,0]
    y=trajectory[:,1]
    z=trajectory[:, 2]
    psi=trajectory[:,3]

    # B-spline curve order
    k = 5
    # Trajectory time
    T = 20
    l=len(x)  
    # knot vector
    t=np.linspace(0,T,l-k+1,endpoint=True)
    t=np.append([0]*k,t)
    t=np.append(t,[T]*k)
    # B-splines
    spline_x = BSpline(t,x,k)
    spline_y = BSpline(t,y,k)
    spline_z = BSpline(t,z,k)
    spline_psi = BSpline(t,psi,k)
    spline_vx = spline_x.derivative(1)
    spline_vy = spline_y.derivative(1)
    spline_vz = spline_z.derivative(1)
    spline_ax = spline_vx.derivative(1)
    spline_ay = spline_vy.derivative(1)
    spline_az = spline_vz.derivative(1)
    spline_wpsi = spline_psi.derivative(1)
    u=np.linspace(0,T,50,endpoint=True)

    xs = spline_x(u)
    ys = spline_y(u)
    zs = spline_z(u)
    psis = spline_psi(u)
    vxs = spline_vx(u)
    vys = spline_vy(u)
    vzs = spline_vz(u)
    wpsis = spline_wpsi(u)

    axs = spline_ax(u)
    ays = spline_ay(u)
    azs = spline_az(u) + 9.8
    thetas = np.arctan2(axs, azs)
    phis = np.arctan2(ays, azs)

    # Header for path publishing
    path_for_pub = Path() 
    path_header = Header()
    path_header.frame_id = "nav"	
    pose_header = Header()
    pose_header.frame_id = "nav"

    poses = []
    for i in range(len(xs)):
        des_pose = array2pose(np.array([xs[i],ys[i],zs[i]]), euler2quat(np.array([0, 0, psis[i]])))
        des_twist = array2twist(np.array([vxs[i],vys[i],vzs[i]]), np.array([0,0,wpsis[i]]))
        path.append((des_pose, des_twist))
        
        stamped_pose = PoseStamped()
        pose_header.seq = pose_header.seq + 1
        stamped_pose.header = pose_header
        stamped_pose.pose = des_pose
        poses.append(stamped_pose)
        
    path_for_pub.header = path_header
    path_for_pub.poses = poses	
    path_publisher.publish(path_for_pub)
    '''
    return path
    

def signal_handler(_, __):
    # enable Ctrl+C of the script
    sys.exit(0)	

def reset_odometry():
    global init_pose, drone_pose
    init_pose_position, init_pose_orientation = pose2array(drone_pose)
    init_euler = quat2euler(init_pose_orientation)
    init_euler[0] = 0
    init_euler[1] = 0
    init_pose_orientation = euler2quat(init_euler)
    init_pose = array2pose(init_pose_position, init_pose_orientation)
    rospy.loginfo("Odometry Reseted ...")

def drone_odometry_callback(drone_odom):
    global drone_pose, drone_vel, init_pose
    drone_pose = drone_odom.pose.pose
    drone_vel = drone_odom.twist.twist
    drone_pose = pose_diff(drone_pose, init_pose)
    drone_pose_publisher.publish(drone_pose)
    
    # Publish stamped pose for rviz
    drone_stamped_pose = PoseStamped()
    pose_header = Header()
    pose_header.frame_id = "nav"
    drone_stamped_pose.header = pose_header
    drone_stamped_pose.pose = drone_pose
    drone_stamped_pose_publisher.publish(drone_stamped_pose)

def gate_pose_callback(gate_pose_wrt_drone):
    global trajectory, drone_pose, AUTONOMY, GATE_IN_VIEW

    drone_position, drone_pose_ornt_arr = pose2array(drone_pose)
    gate_position_wrt_drone, gate_pose_wrt_drone_ornt_arr = pose2array(gate_pose_wrt_drone)

    # Check if gate_pose is not empty and not none
    if not np.array_equal(gate_pose_wrt_drone_ornt_arr , np.zeros(4)) and not (gate_pose_wrt_drone is None):
        #print("Got Gate Pose. Replanning trajectory.")
        gate_pose_ornt_arr = tfs.quaternion_multiply(gate_pose_wrt_drone_ornt_arr , drone_pose_ornt_arr)		
        gate_position_wrt_ground = drone_position + qv_mult( drone_pose_ornt_arr, gate_position_wrt_drone)		
        gate_pose = array2pose(gate_position_wrt_ground, gate_pose_ornt_arr)		
        gate_pose_ground_publisher.publish(gate_pose)
        
        # Publish stamped pose for rviz
        gate_pose_ground_stamped = PoseStamped()
        pose_header = Header()
        pose_header.frame_id = "nav"
        gate_pose_ground_stamped.header = pose_header
        gate_pose_ground_stamped.pose = gate_pose
        gate_pose_ground_stamped_publisher.publish(gate_pose_ground_stamped)
        
        #rospy.loginfo("[Gate Pose] {}".format(gate_pose))
        # Generate path through the gate
        if not GATE_IN_VIEW:
            GATE_IN_VIEW = True
            rospy.sleep(3.0)
        
        if AUTONOMY:
            trajectory.setPath(generatePath(gate_position_wrt_ground))
    else:
        GATE_IN_VIEW = False


def autonomy_callback(empty):
    global trajectory, AUTONOMY	, SIMULATION
    if not AUTONOMY:
        print("Drone in autonomus mode...")
        AUTONOMY = True
        
        if not SIMULATION:
            # Set the path
            path = generatePath([0,0,0])
            trajectory.setPath(path)
            
        # takeoff
        #takeoff_publisher.publish(Empty())
    else:
        print("Drone in manual mode...")
        AUTONOMY = False
    


def pub_traj_error():
    global trajectory, target_pose, drone_pose, drone_vel

    target_pose = trajectory.current_point[0]
    target_vel = trajectory.current_point[1]

    target_position, target_orient_arr = pose2array(target_pose)
    drone_position, drone_orient_arr = pose2array(drone_pose)


    # Calculate Position error in body frame
    position_error = target_position - drone_position					   
    position_error = qv_mult( tfs.quaternion_conjugate(drone_orient_arr), position_error )	

    # Calculate Orientation error
    pose_error_orient_arr = tfs.quaternion_multiply( target_orient_arr , tfs.quaternion_conjugate(drone_orient_arr) )

    # Pose error	
    pose_error = array2pose(position_error, pose_error_orient_arr)

    # Calculate Linear Velocity error in body frame
    twist_error = twist_diff(target_vel, drone_vel)
    twist_lin_err, twist_ang_arr = twist2array(twist_error)
    twist_lin_err = qv_mult( tfs.quaternion_conjugate(drone_orient_arr), twist_lin_err )
    twist_error = array2twist(twist_lin_err, twist_ang_arr)

    #Find the magnitude of the error between the target pose and the drone pose
    #pose_error_magnitude = np.linalg.norm(position_error) + 5*np.linalg.norm(pose_error_orient_arr - np.array([0,0,0,1]))
    #trajectory.check_trajectory_error(pose_error_magnitude)

    # Make the nearest trajectory point, the target point
    trajectory.update_current_point(drone_position)

    traj_error = Traj_error()
    traj_error.pose_error = pose_error
    traj_error.twist_error = twist_error

    rospy.loginfo("Current Point: {}".format(trajectory.prev_current_point))
    traj_error_publisher.publish(traj_error)
    pose_target_publisher.publish(target_pose)
        
        
if __name__ == '__main__':
    # Initialize node
    rospy.init_node('planner', anonymous=False)
    signal.signal(signal.SIGINT, signal_handler)

    # Subcribers
    # State subscription


    # Set simulation parameter. 
    SIMULATION = rospy.get_param("/simulation")

    # Drone odometry subscription
    if SIMULATION:
        rospy.Subscriber('/ground_truth/state', Odometry, drone_odometry_callback)
        takeoff_publisher = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)	
        land_publisher = rospy.Publisher('/ardrone/land', Empty, queue_size=1)	
    else:
        rospy.Subscriber('/bebop/odom', Odometry, drone_odometry_callback)
        #takeoff_publisher = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)	
        #land_publisher = rospy.Publisher('/bebop/land', Empty, queue_size=1)
        
    # Gate Pose wrt drone
    #rospy.Subscriber('/gate_pose', Pose, gate_pose_callback)

    # Filtered Gate Pose wrt drone
    rospy.Subscriber('/filtered_gate_pose', Pose, gate_pose_callback)

    # autonomy subscription
    rospy.Subscriber('/start_autonomy', Empty, autonomy_callback)

    # Publishers
    # traj_error publication
    traj_error_publisher = rospy.Publisher('/traj_error', Traj_error, queue_size=5)

    # publish corrected drone pose
    drone_pose_publisher = rospy.Publisher('/drone_pose', Pose, queue_size=5)

    # publish path
    path_publisher = rospy.Publisher('/path', Path, queue_size=5)

    # publish drone pose (stamped) for rviz
    drone_stamped_pose_publisher = rospy.Publisher('/drone_stamped_pose', PoseStamped, queue_size=5)

    # publish current target pose for debugging
    pose_target_publisher = rospy.Publisher('/pose_target', Pose, queue_size=5)

    # publish gate pose wrt ground for debugging
    gate_pose_ground_publisher = rospy.Publisher('/gate_pose/ground', Pose, queue_size=5)
    
    # publish gate pose ground (stamped) for rviz
    gate_pose_ground_stamped_publisher = rospy.Publisher('/gate_pose/ground/stamped', PoseStamped, queue_size=5)

    # publish AUTONOMY state for debugging
    autonomy_state_publisher = rospy.Publisher('/autonomyState', Bool, queue_size=2)

    # publish SIMULATION for debugging
    sim_state_publisher = rospy.Publisher('/simulationState', Bool, queue_size=2)

    # Update rate for the planner loop
    rate = rospy.Rate(50) # 50 hz

    reset_odometry()

    while not rospy.is_shutdown():		
        if AUTONOMY:			
            if trajectory.prev_current_point < len(trajectory.path):
                # cycle through the trajectory
                pub_traj_error()
                print("Tracking the path ...")
            #elif (trajectory.prev_current_point >= len(trajectory.path)-1) and not trajectory.cycle:
                ## Land and go to manual mode
                #land_publisher.publish(Empty())
                #print("Landing ...")
                #AUTONOMY = False
                pass
        
        autonomy_state_publisher.publish(AUTONOMY)
        sim_state_publisher.publish(SIMULATION)
        rate.sleep()

