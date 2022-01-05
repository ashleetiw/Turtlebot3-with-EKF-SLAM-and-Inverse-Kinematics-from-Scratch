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
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include<string>
#include <armadillo>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "nuslam/kalman.hpp"

// GLOBAL VARS
std::string odom_id_, body_id_;
float wheel_base, wheel_radius, frequency;

float left;
float right;
rigid2d::Twist2D twist;
rigid2d::DiffDrive drive_model;
rigid2d::Pose pose;
std::vector<rigid2d::Vector2D> arr;

// float x=0;
// float y=0;


void joint_callback(const sensor_msgs::JointState::ConstPtr &joint)
{
  
 
    left = joint->position.at(0);
    right = joint->position.at(1);

}

void sensor_callback(const visualization_msgs::MarkerArray &landmarks){
  rigid2d::Vector2D v;
  for(unsigned int i = 0; i<landmarks.markers.size();i++)
  {
    v.x=landmarks.markers[i].pose.position.x;
    v.y=landmarks.markers[i].pose.position.y;
    arr.push_back(v);
  }  
  
}

int main(int argc, char** argv)
/// The Main Function ///
{
  
    ros::init(argc, argv, "slam"); // register the node on ROS

    ros::NodeHandle nh; // PUBLIC handle to ROS
    // Init Private Parameters

    nh.getParam("odom_frame_id", odom_id_);
    nh.getParam("body_frame_id", body_id_);
    // Init Global Parameters
    nh.getParam("wheel_base", wheel_base);
    nh.getParam("wheel_radius", wheel_radius);
    frequency = 60;

    // Init Service Server
    // ros::ServiceServer set_pose_server = nh.advertiseService("setpose", service_callback);
    ros::Subscriber sub = nh.subscribe("joint_states", 1, joint_callback);
    ros::Subscriber sub_sensor = nh.subscribe("fake_sensor", 1, sensor_callback);
    // Init Publisher
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("slam_map", 10);
    ros::Publisher slam_path_pub = nh.advertise<nav_msgs::Path>("slam_path", 10);
    ros::Publisher odom_path_pub = nh.advertise<nav_msgs::Path>("odom_path", 10);

    // Init Transform Broadcaster
    static tf2_ros::TransformBroadcaster odom_broadcaster;
    tf2_ros::TransformBroadcaster map_broadcaster;
    tf2_ros::TransformBroadcaster world_broadcaster;
    // path from odometry
    nav_msgs::Path odom_path;
    // path from SLAM
    nav_msgs::Path ekf_path;

    // Init Time

    pose.theta = 0;
    pose.x = 0;
    pose.y = 0;

    // robot model
    drive_model= rigid2d::DiffDrive(pose, wheel_base, wheel_radius);

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

        rigid2d::WheelVel vel=drive_model.update_Odom(left, right,wheel_radius,wheel_base);
        pose = drive_model.getpose();

        
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
        // l vel = drive_model.wheelvel();
        rigid2d::Twist2D twist = drive_model.wheelsToTwist(vel,wheel_radius,wheel_base);

        ROS_ERROR_STREAM(pose.x);

        nuslam::EKF ekf(5);
        arma::rowvec state=ekf.slam(twist,arr);
         rigid2d::Transform2D Tmr = ekf.getRobotState();
                


        //         // // transfrom map to odom      

                //  // odom to robot
                rigid2d::Vector2D Vor = rigid2d::Vector2D(pose.x, pose.y);
                rigid2d::Transform2D Tor = rigid2d::Transform2D(Vor, pose.theta);
               
                // Now find Tmo
                rigid2d::Transform2D Tmo = Tmr*Tor.inv();

            
                geometry_msgs::TransformStamped mo;
                mo.header.stamp = ros::Time::now();

                mo.header.frame_id = "map";
                mo.child_frame_id = "odom";
                mo.transform.translation.x = Tmo.dx;
                mo.transform.translation.y = Tmo.dy;
                mo.transform.translation.z = 0;
                // use tf2 to create transform
                tf2::Quaternion q1;
                q1.setRPY(0, 0, Tmo.angle);
                geometry_msgs::Quaternion odom_quat2 = tf2::toMsg(q1);
                mo.transform.rotation = odom_quat2;
                // // Send the Transform
                map_broadcaster.sendTransform(mo);



                geometry_msgs::TransformStamped wm;
                wm.header.stamp = ros::Time::now();

                wm.header.frame_id = "world";
                wm.child_frame_id = "map";
                wm.transform.translation.x = 0;
                wm.transform.translation.y = 0;
                wm.transform.translation.z = 0;
                // use tf2 to create transform
                tf2::Quaternion q2;
                q2.setRPY(0, 0, 0);
                geometry_msgs::Quaternion odom_quat3 = tf2::toMsg(q2);
                wm.transform.rotation = odom_quat3;
                // // Send the Transform
                world_broadcaster.sendTransform(wm);


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

        // ROS_ERROR_STREAM(twist.linear_vx);

                std::vector<rigid2d::Vector2D> map;
                ekf.getLandmark(map,5);

                visualization_msgs::MarkerArray marker_array;
                marker_array.markers.resize(map.size());
                
                for(unsigned int i = 0;i < map.size();i++)
                {
                  marker_array.markers[i].header.frame_id = "world";
                  marker_array.markers[i].header.stamp = ros::Time::now();
                  marker_array.markers[i].lifetime = ros::Duration(1.0/ 5.0); // 1/5th sec
                  marker_array.markers[i].ns = "marker";
                  marker_array.markers[i].id = i;

                  marker_array.markers[i].type = visualization_msgs::Marker::CYLINDER;
                  marker_array.markers[i].action = visualization_msgs::Marker::ADD;

                  marker_array.markers[i].pose.position.x =  map.at(i).x;
                  marker_array.markers[i].pose.position.y =  map.at(i).y;
                  marker_array.markers[i].pose.position.z = 0.15;

                  marker_array.markers[i].pose.orientation.x = 0.0;
                  marker_array.markers[i].pose.orientation.y = 0.0;
                  marker_array.markers[i].pose.orientation.z = 0.0;
                  marker_array.markers[i].pose.orientation.w = 1.0;

                  marker_array.markers[i].scale.x = 2.0 * 0.05;
                  marker_array.markers[i].scale.y = 2.0 * 0.05;
                  marker_array.markers[i].scale.z = 0.1;

                  marker_array.markers[i].color.r = 0.0f;
                  marker_array.markers[i].color.g = 0.0f;
                  marker_array.markers[i].color.b = 1.0f;
                  marker_array.markers[i].color.a = 1.0f;
                
                }
                marker_pub.publish(marker_array);


        // path from SLAM
        geometry_msgs::PoseStamped slam_pose;
        tf2::Quaternion q_mr;
        q_mr.setRPY(0, 0, Tmr.angle);
        geometry_msgs::Quaternion quat_mr;
        quat_mr = tf2::toMsg(q_mr);
        slam_pose.header.stamp = ros::Time::now();
        slam_pose.pose.position.x = Tmr.dx;
        slam_pose.pose.position.y = Tmr.dy;
        slam_pose.pose.orientation = quat_mr;

        ekf_path.header.stamp = ros::Time::now();
        ekf_path.header.frame_id = "world";
        ekf_path.poses.push_back(slam_pose);
        slam_path_pub.publish(ekf_path);

        /////////////////////////////////////////////////////////////////////////////

        // path from odom
        geometry_msgs::PoseStamped odom_pose;

        tf2::Quaternion q_or;
        q_or.setRPY(0, 0, pose.theta);
        geometry_msgs::Quaternion quat_or;
        quat_or = tf2::toMsg(q_or);

        odom_pose.header.stamp = ros::Time::now();
        odom_pose.pose.position.x = pose.x;
        odom_pose.pose.position.y = pose.y;
        odom_pose.pose.orientation = quat_or;

        odom_path.header.stamp = ros::Time::now();
        odom_path.header.frame_id = "world";

        odom_path_pub.publish(odom_path);

        rate.sleep();
    }

  return 0;
}
  ////////////  slam implemntation ////////////////
      
            // arma::rowvec state=ekf.odomupdate(twist);
            // arma::mat sigma_bar;
            // sigma_bar=arma::mat(13,13,arma::fill::zeros);
            // sigma_bar=ekf.predict(twist,sigma_bar);
            // std::vector<nuslam::landmark> lm(arr.size());

            // ekf.changeframe(arr,lm);
            // // ROS_ERROR_STREAM(lm.size());

            //  arma::mat H;
            //   H=arma::mat(2,13,arma::fill::zeros);

            //   arma::mat K;
            //   K=arma::mat(13,2,arma::fill::zeros);
            //    arma::rowvec z_hat = ekf.predictedLandmark(0,state);
            // //    H=ekf.measurementJacobian(0, state, H);
            // //    arma::mat temp=H*sigma_bar*trans(H) ;   // +R
            
            // //       K= sigma_bar * trans(H)*inv(temp); 
            // //   arma::rowvec delta;
            // //       delta= arma::rowvec(2,arma::fill::zeros);
            // //       delta(0)=lm.at(0).x-z_hat(0);
            // //       delta(1)=rigid2d::normalize_angle(lm.at(0).y-z_hat(1));

















          //   arma::rowvec state_bar;
          //   state_bar=arma::rowvec(13,arma::fill::zeros);
          //   state_bar=ekf.odomupdate(twist,state_bar);

          //   arma::mat sigma_bar;
          //   sigma_bar=arma::mat(13,13,arma::fill::zeros);
          //   sigma_bar=ekf.predict(twist,sigma_bar);

          //   // vector2d has x,y  now incorporate r and m using struct landmark
          //   std::vector<nuslam::landmark> lm(arr.size());
          //   ekf.changeframe(arr,lm);

          //   arma::mat H;
          //  H=arma::mat(2,13,arma::fill::zeros);

          // arma::mat K;
          // K=arma::mat(13,2,arma::fill::zeros);

          // arma::rowvec z_hat = ekf.predictedLandmark(0,state_bar);
          // H=ekf.measurementJacobian(0, state_bar, H);

          // arma::mat R;
          //         // init measurement noise
          // R = arma::mat(2,2,arma::fill::zeros);
          // R(0,0) = 1e-10;   // r var
          // R(1,1) = 1e-10;   // b var

          // arma::mat temp=H*sigma_bar*trans(H)+R ; 

      
          //   ROS_ERROR_STREAM(inv(temp));