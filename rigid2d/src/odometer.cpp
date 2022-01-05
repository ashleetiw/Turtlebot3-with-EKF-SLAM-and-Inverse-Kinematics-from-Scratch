/// \file
/// \brief Main: Publishes odom messages for diff drive robot based on wheel joint states
///
/// PARAMETERS:
///   odom_id_ (string): parent frame for  tf transform
///   body_id_ (string): child frame for tf transform
///   wheel_base (float): wheel base of modeled diff drive robot
///   wheel_radius (float): wheel radius of modeled diff drive robot
///   frequency (int): frequency of control loop.
///

///
///   odom_tf (geometry_msgs::TransformStamped): odometry frame transform used to update RViz sim
///   odom (nav_msgs::Odometry): odometry message containing pose and twist published to odom topic
///
/// PUBLISHES:
///   odom (nav_msgs::Odometry): publishes odometry message containing pose(x,y,z) and twist(lin,ang)
/// SUBSCRIBES:
///   /joint_states (sensor_msgs::JointState), which records the ddrive robot's joint states
///
/// FUNCTIONS:
///   joint_callback (void): callback for /joint_states subscriber, which records the ddrive robot's joint states
///   service_callback(bool): callback for setpose service, which resets the robot's pose in the tf tree

#include <ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "rigid2d/setpose.h"
#include <iterator>
#include <random>

#include<string>

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

// GLOBAL VARS
std::string odom_id_, body_id_;
float wheel_base, wheel_radius, frequency;

float left;
float right;
rigid2d::Twist2D twist;
rigid2d::DiffDrive drive_model;
rigid2d::Pose pose;

// float x=0;
// float y=0;


void joint_callback(const sensor_msgs::JointState::ConstPtr &joint)
{
    left = joint->position.at(0);
    right = joint->position.at(1);

}

bool service_callback(rigid2d::setpose::Request& req, rigid2d::setpose::Response& res)
/// \brief setpose service callback. Sets the turtlebot's pose belief to desired value.
///
/// \param x (float32): desired x pose.
/// \param y (float32): desired y pose.
/// \param theta (float32): desired theta pose.
/// \returns result (bool): True or False.
{
    pose.theta  = req.theta; 
    pose.x      = req.x;
    pose.y      = req.y;
    drive_model.setpose(pose);

  return true;
}

int main(int argc, char** argv)
/// The Main Function ///
{
  
    ros::init(argc, argv, "odometer"); // register the node on ROS

    ros::NodeHandle nh; // PUBLIC handle to ROS
    // Init Private Parameters

    nh.getParam("odom_frame_id", odom_id_);
    nh.getParam("body_frame_id", body_id_);
    // Init Global Parameters
    nh.getParam("wheel_base", wheel_base);
    nh.getParam("wheel_radius", wheel_radius);
    frequency = 60;

    // Init Service Server
    ros::ServiceServer set_pose_server = nh.advertiseService("setpose", service_callback);
    // Init Subscriber
    ros::Subscriber sub = nh.subscribe("joint_states", 1, joint_callback);
    // Init Publisher
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
    // Init Transform Broadcaster
    static tf2_ros::TransformBroadcaster odom_broadcaster;

    pose.theta = 0;
    pose.x = 0;
    pose.y = 0;

    // robot model
    drive_model= rigid2d::DiffDrive();

    // ROS_ERROR_STREAM(pose.x);

    // timer
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    left = 0; 
    right = 0;

    ros::Rate rate(frequency);

    // Main While
    while (ros::ok())
    {

        ros::spinOnce();
        current_time = ros::Time::now();
        
        // rigid2d::Encoders encoders=drive_model.getEncoders();
        // ROS_ERROR_STREAM(encoders.leftangle);
        // ROS_ERROR_STREAM(left);

        rigid2d::WheelVel wheel=drive_model.update_Odom(left,right,wheel_radius,wheel_base);

        pose = drive_model.getpose();
        // ROS_ERROR_STREAM(pose.y);
        

        // transformation 
        geometry_msgs::TransformStamped odom_tf;
        odom_tf.header.stamp = current_time;
        odom_tf.header.frame_id = odom_id_;
        odom_tf.child_frame_id = body_id_;
        // Pose
        odom_tf.transform.translation.x = pose.x;
        odom_tf.transform.translation.y = pose.y;
        odom_tf.transform.translation.z = 0;
        // use tf2 to create transform
        tf2::Quaternion q;
        q.setRPY(0, 0, pose.theta);
        geometry_msgs::Quaternion odom_quat = tf2::toMsg(q);
        odom_tf.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_tf);

        // get robots velocity 
        rigid2d::WheelVel vel = drive_model.wheelvel();
        rigid2d::Twist2D twist = drive_model.wheelsToTwist(vel,wheel_radius,wheel_base);
        // odom msg
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = odom_id_;
        // Pose
        odom.pose.pose.position.x = pose.x;
        odom.pose.pose.position.y = pose.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        // Twist
        odom.child_frame_id = body_id_;
        odom.twist.twist.linear.x = twist.linear_vx;
        odom.twist.twist.linear.y = twist.linear_vy;
        odom.twist.twist.angular.z = twist.angle_z;
        pub.publish(odom);

        rate.sleep();
    }

  return 0;
}
