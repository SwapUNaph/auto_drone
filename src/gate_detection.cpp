#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <auto_drone/Drone_Pose.h>
#include <auto_drone/WP_Msg.h>
#include <auto_drone/Detection_Acive.h>
#include <nav_msgs/Odometry.h>

#include <cv_bridge/cv_bridge>
#include <opencv2/imgproc/imgproc.h>

#include <Eigen/Dense>

using namespac Eigen;

//*************** Global Variables **************

 Vector3f drone_position;
 Vector3f drone_orientation;
 Vector3f drone_linear_vel;
 Vector3f drone_angular_vel;
 
 //**********************************************

void drone_pose_callback(const nav_msgs::Odometry::ConstPtr& odom){
   drone_position << odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z;
   Quaternion drone_quat();
}

int main(int argc, char** argv){
    
    //Node handle
    ros::init(argc, argv, "gate_detect");
    ros::NodeHandle nh;

    //Publishers
    ros::Publisher raw_gate_WP_publisher = nh.advertise<auto_drone::WP_Msg>("/auto/raw_gate_WP");

    ros::Publisher filtered_gate_WP_publisher = nh.advertise<auto_drone::WP_Msg>("/auto/filtered_gate_WP");


    //Subscribers
    //ros::Subscriber sub1 = nh.subscribe("/auto/pose", 5, drone_odometry_callback);
    ros::Subscriber sub1 = nh.subscribe("/bebop/odom", 5, drone_odometry_callback);
    
    ros::Rate loop_rate(30);
    
    while(ros::ok()){
	
	loop_rate.sleep();
    }
    
    return 0;
}

