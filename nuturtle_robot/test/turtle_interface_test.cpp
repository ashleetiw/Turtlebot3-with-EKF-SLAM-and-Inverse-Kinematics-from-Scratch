/// \file 
/// \brief ROS node for testing basic ROS setup and serivces using rostest 

/// PUBLISHERS: /cmd_vel: Twist for robot to perform
///  	      /sensor_data: encoder  data
 
///SUBSCRIBERS: /wheel_cmd: Scalar specyfing motor behavior/speed  
///  	       /joint_states: Current state of the robot's joints 
#define CATCH_CONFIG_RUNNER
#include <ros/ros.h>
#include <catch_ros/catch.hpp>
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <cmath>
#include <stdlib.h>
#include "std_msgs/String.h"
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

static ros::Publisher cmd_vel_pub;
static ros::Subscriber wheel_cmd_sub;

static ros::Publisher sensor_pub;
static ros::Subscriber joint_states_sub;

static nuturtlebot::WheelCommands observed_wheel_cmd;
static sensor_msgs::JointState observed_joint_state;

static bool waitForTurtleInterface = false;
static int ticks_per_rotation=4096;

/// \brief  subscribes to "/wheel_cmd" topic 
/// \param wheel_cmd - a command for the wheels to use 
void wheel_cmd_callback(nuturtlebot::WheelCommands wheel_cmd) {
	
	observed_wheel_cmd = wheel_cmd;
	// waitForTurtleInterface = false;
}


/// \brief  subscribes to "/joint_states" topic 
/// \param joint_state - a ROS message describing the robot's joints current state
void joint_states_callback(sensor_msgs::JointState joint_state) {

        observed_joint_state = joint_state;
        
        // waitForTurtleInterface = false;
}

// /* Publish a translation on /cmd_vel, wait for
//  * the subscriber to publish back the corresponding
//  * wheel_cmd and verify that it is the correct wheel command */
TEST_CASE("translate") {
	
	geometry_msgs::Twist translateOnly;
	translateOnly.linear.x = 1.0;
	
	// waitForTurtleInterface = true;
	cmd_vel_pub.publish(translateOnly);
	
	//ros::spinOnce();	
	while (waitForTurtleInterface) {
		ros::spinOnce();
	}
	
	// waitForTurtleInterface = false;
	REQUIRE(round(observed_wheel_cmd.left_velocity)== round(observed_wheel_cmd.right_velocity));	
}

/* Publish a rotation on /cmd_vel, wait for
 * the subscriber to publish back the corresponding
 * wheel_cmd and verify that it is the correct wheel command */
TEST_CASE("rotate") {
	
	geometry_msgs::Twist rotateOnly;
        rotateOnly.angular.x = 0.0;
        rotateOnly.angular.y = 0.0;
        rotateOnly.angular.z = 1.0;

	// waitForTurtleInterface = true;
        cmd_vel_pub.publish(rotateOnly);

        //ros::spinOnce();      
        while (waitForTurtleInterface) {
                ros::spinOnce();
        }

        // waitForTurtleInterface = false;
        REQUIRE(round(observed_wheel_cmd.left_velocity)==round(-1 * observed_wheel_cmd.right_velocity));
}


/* Verifies that encoder data on sensors
 * is converted to joint_states properly */
TEST_CASE("encoders") {
	
	using namespace rigid2d;

	// Publish fake encoder data that indicates we rotated exactly 1/4 rotation
	// Verify that the angle is +/-PI/2
	nuturtlebot::SensorData sensor_data1;
	nuturtlebot::SensorData sensor_data2;
	
	sensor_data1.left_encoder = 0;
	sensor_data1.right_encoder = 0;

	// waitForTurtleInterface = true;
	sensor_pub.publish(sensor_data1);
	while (waitForTurtleInterface) {
                ros::spinOnce();
        }
	
	sensor_data2.left_encoder = ticks_per_rotation / 4;
        sensor_data2.right_encoder = ticks_per_rotation / 4;
	
	// waitForTurtleInterface = true;
        sensor_pub.publish(sensor_data2);

        while (waitForTurtleInterface) {
                ros::spinOnce();
        }

        waitForTurtleInterface = false;

        REQUIRE(sensor_data2.left_encoder==ticks_per_rotation / 4);
	REQUIRE(sensor_data1.right_encoder==0);
}

/// \brief  Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  ros::init(argc, argv, "turtle_interface_test");
  ros::NodeHandle n;
  Catch::Session session;

  // true makes it a latched publisher
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);    
  sensor_pub = n.advertise<nuturtlebot::SensorData>("sensor_data", 1, true);

  wheel_cmd_sub = n.subscribe("wheel_cmd", 1, wheel_cmd_callback); 	
  joint_states_sub = n.subscribe("joint_states", 1, joint_states_callback); 
	
  return session.run();
}