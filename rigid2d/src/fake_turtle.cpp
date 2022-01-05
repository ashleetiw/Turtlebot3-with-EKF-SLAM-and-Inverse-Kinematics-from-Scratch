/// \file
/// \brief Main: subscribers to cmd Messages from turtlsim_teleop_key  and publishes encoder messgea to /joint_states
///
/// PARAMETERS:
///   left_wheel (string): the left wheel joint name in diff_drive.urdf.xacro of nuturtle_desctipion pkg
///   right_wheel (string): the right wheel joint name in diff_drive.urdf.xacro of nuturtle_desctipion pkg
///   wheel_base (float): wheel base of modeled diff drive robot
///   wheel_radius (float): wheel radius of modeled diff drive robot
///   frequency (int): frequency of control loop.

///
/// PUBLISHES:
///   joint (sensor_msgs::JointState): publishes joint state message containing left and right wheel angles
/// SUBSCRIBES:
///   /cmd_vel (geometry_msgs::Twist): subscriber, which records the commanded twist
///
/// FUNCTIONS:
///   vel_callback (void): callback for /cmd_vel subscriber, which records the commanded twist

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include<sensor_msgs/JointState.h>
#include<sensor_msgs/JointState.h>

#include<string>

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

// GLOBAL VARS
std::string odom_id, body_id, left_wheel, right_wheel;
double wheel_base, wheel_radius;
// rigid2d::WheelVelocities w_ang;
rigid2d::Encoders encoders;
rigid2d::Pose pose;
rigid2d::DiffDrive drive_model;
rigid2d::Twist2D Vb;      // body cmd_vel
double frequency = 60;
bool flag= false;

  

void vel_callback(const geometry_msgs::Twist &tw)
{ 
  /// \brief cmd_vel subscriber callback. Records commanded twist
  ///
  /// \param tw (geometry_msgs::Twist): the commanded linear and angular velocity
  /// \returns wheel (rigid2d::wheel_vel): the left and right wheel angles
  Vb.angle_z=tw.angular.z/frequency;
  Vb.linear_vx=tw.linear.x/frequency;
  Vb.linear_vy=tw.linear.y/frequency;

}

int main(int argc, char** argv)
/// The Main Function ///
{
  

  ros::init(argc, argv, "fake_turtle"); // register the node on ROS
  ros::NodeHandle nh; // PUBLIC handle to ROS
  // Parameters
  nh.getParam("left_wheel_joint", left_wheel);
  nh.getParam("right_wheel_joint", right_wheel);
  nh.getParam("wheel_base", wheel_base);
  nh.getParam("wheel_radius", wheel_radius);


  // Init Subscriber
  ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 1, vel_callback);
  // Init Publisher
  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

      // init pose to zero 
  pose = {0,0,0};
  Vb = {0,0,0};
  // Init Time
  ros::Time current_time;
  current_time = ros::Time::now();

  ros::Rate rate(frequency);

   // robot model
  drive_model= rigid2d::DiffDrive(pose,wheel_radius,wheel_base);
  // Main Whil
  while (ros::ok())
  {
  	ros::spinOnce();

    drive_model.forward(Vb,wheel_radius,wheel_base);
    encoders = drive_model.getEncoders();

    current_time = ros::Time::now();
   
   
    sensor_msgs::JointState joint;

    joint.header.stamp = current_time;

      // joint stores vectors, so we push back the name corresp. to left wheel joint
      joint.name.push_back(left_wheel);
      // then we insert the left wheel encoder value
      joint.position.push_back(encoders.leftangle);

      // repeat with right wheel. Note order must be consistent between name pushback and
      // encoder value pushback
      joint.name.push_back(right_wheel);
      joint.position.push_back(encoders.rightangle);

      joint_pub.publish(joint);


      rate.sleep();

  }

  return 0;
}