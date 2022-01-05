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
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/Point.h>

// GLOBAL VARS
std::string odom_id_, body_id_;
double wheel_base, wheel_radius, frequency;

double left,right; // angles for odom
double ekf_left, ekf_right; // angles for slam
rigid2d::Twist2D twist;
rigid2d::DiffDrive drive_odom,drive_slam;
rigid2d::Pose pose,pose_ekf;
std::vector<rigid2d::Vector2D> arr;
bool slam_update;
bool odom_update;
bool start=false;
static bool unknown_data_association=false;

// float x=0;
// float y=0;
void path_callback(const nav_msgs::Path &p){
        if (p.poses[0].pose.position.x!=0){
          start=true;
        }
}

void joint_callback(const sensor_msgs::JointState::ConstPtr &joint)
{
    left = joint->position.at(0);
    right = joint->position.at(1);

    odom_update=true;
}


void sensor_callback(const visualization_msgs::MarkerArray &landmarks){
  // std::vector<rigid2d::Vector2D> arr;
  rigid2d::Vector2D v;
  arr.clear();  // clears the landmarks so that the size doesn't increase everytime
  // ROS_ERROR_STREAM("in here");
  for(unsigned int i = 0; i<5;i++)
  {
    v.x=landmarks.markers[i].pose.position.x;
    v.y=landmarks.markers[i].pose.position.y;
    
    arr.push_back(v);
  }  
  slam_update=true;
  
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
    frequency = 100;
    odom_update = false;
    slam_update = false;

    // timer
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    left = 0; 
    right = 0;

    ros::Rate rate(10);

    ros::Subscriber sub_sensor = nh.subscribe("/fake_sensor",1, sensor_callback);
    // Init Subscriber
    ros::Subscriber sub = nh.subscribe("/joint_states", 1, joint_callback);

    //  ros::Subscriber sub_path = nh.subscribe("/real_path", 1,path_callback);
    // Init Publisher
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("slam_map", 1);
    ros::Publisher slam_path_pub = nh.advertise<nav_msgs::Path>("slam_path", 1);
    ros::Publisher odom_path_pub = nh.advertise<nav_msgs::Path>("odom_path", 1);
    ros::Publisher error = nh.advertise<geometry_msgs::Point>("error", 1);
    // Init Transform Broadcaster
    tf2_ros::TransformBroadcaster odom_broadcaster;
    tf2_ros::TransformBroadcaster map_broadcaster;
    static tf2_ros::StaticTransformBroadcaster  world_broadcaster;

     //                               WORLD TO MAP
       geometry_msgs::TransformStamped wm;

  wm.header.stamp = ros::Time::now();
  wm.header.frame_id = "world";
  wm.child_frame_id ="map";
  wm.transform.translation.x = 0;
  wm.transform.translation.y = 0;
  wm.transform.translation.z = 0;
  wm.transform.rotation.x = 0;
  wm.transform.rotation.y = 0;
  wm.transform.rotation.z = 0;
  wm.transform.rotation.w = 1;  
  world_broadcaster.sendTransform(wm); 
    
    
    pose.theta = 0;
    pose.x = 0;
    pose.y = 0;


    drive_odom= rigid2d::DiffDrive();
    drive_slam= rigid2d::DiffDrive();
    nuslam::EKF ekf(5);
    nav_msgs::Path odom_path;
    nav_msgs::Path ekf_path;


  ///////////////////////////////////////////////////////////////////////////////////
    // ROS_ERROR_STREAM(arr.size());

    // Main While
    while (ros::ok())
    {

      ros::spinOnce();
      current_time = ros::Time::now();
      
      if (odom_update){
            rigid2d::WheelVel wo=drive_odom.update_Odom(left,right,wheel_radius,wheel_base);
            pose = drive_odom.getpose();

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



                ekf_left=left;
                ekf_right=right;

                if(slam_update==true){

                // slam update ( does not alwyas happen )
                rigid2d::WheelVel ws=drive_slam.update_Odom(ekf_left,ekf_right,wheel_radius,wheel_base);

                // rigid2d::WheelVel vel = drive_slam.wheelvel();
                rigid2d::Twist2D twist = drive_slam.wheelsToTwist(ws,wheel_radius,wheel_base);
                ////////////////////////////////////////////////////////
                // arma::rowvec w ;
                // w=arma::rowvec(13 ,arma::fill::zeros);
                // w=ekf.odomupdate(twist);
                // ROS_ERROR_STREAM(w(1));
                // ROS_ERROR_STREAM(w(2));



                ///////////////////////////////////////////
                pose_ekf = drive_slam.getpose();
                twist.linear_vx= twist.linear_vx;
                twist.linear_vy= twist.linear_vy;
                twist.angle_z= twist.angle_z;

                if (unknown_data_association)
                {
                    int md_max = 1e7;//0.30;
                    int md_min = 20000;//0.05;
                       arma::rowvec state=ekf.unknownCorrespondenceSLAM(twist,arr,md_max);
                  }

                  else
                  {
                    arma::rowvec state=ekf.slam(twist,arr);
                  }
                          
                
                // ROS_ERROR_STREAM(state(0));
                slam_update=false;
                }
         


          //                  ODOM PATH
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
          odom_path.header.frame_id = "map";
          odom_path.poses.push_back(odom_pose);

          odom_path_pub.publish(odom_path);

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

        
        odom_update=false;
      }
 
    // //   // //////////////////////   transformations /////////////////////////////////////


      //                                MAP TO ODOM
      pose = drive_odom.getpose();
      rigid2d::Transform2D Tmr = ekf.getRobotState();
      // transform from odom to robot
      rigid2d::Transform2D Tor;
      Tor.dx=pose.x;
      Tor.dy=pose.y;
      Tor.angle=pose.theta;
      // transform from robot to odom
      rigid2d::Transform2D Tmo = Tmr*Tor.inv();
      
      geometry_msgs::TransformStamped mo;
      mo.header.stamp = ros::Time::now();
      mo.header.frame_id = "map";
      mo.child_frame_id = "odom";
      mo.transform.translation.x = Tmo.dx;
      mo.transform.translation.y = Tmo.dy;
      mo.transform.translation.z = 0;
      tf2::Quaternion q1;
      q1.setRPY(0, 0, Tmo.angle);
      geometry_msgs::Quaternion odom_quat2 = tf2::toMsg(q1);
      mo.transform.rotation = odom_quat2;
      map_broadcaster.sendTransform(mo);

        
        //////////////////////////////   paths //////////////////////

            //                   SLAM PATH 
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
    ekf_path.header.frame_id = "map";
    ekf_path.poses.push_back(slam_pose);
    slam_path_pub.publish(ekf_path);





    //erroor.
    geometry_msgs::Point p;
    p.x=pose.x-Tmr.dx;
    p.y=pose.y-Tmr.dy;
    p.z=pose.theta-Tmr.angle;
    error.publish(p);
    /////////////////////////////////PUBLSIHERS ///////////////////////////

      std::vector<rigid2d::Vector2D> map;
      map=ekf.getLandmark(map,5);


      visualization_msgs::MarkerArray marker_array;
      marker_array.markers.resize(map.size());
      
      for(unsigned int i = 0;i < map.size();i++)
      {
        marker_array.markers[i].header.frame_id = "map";
        marker_array.markers[i].header.stamp = ros::Time::now();
        marker_array.markers[i].lifetime = ros::Duration(1.0/ 5.0); // 1/5th sec
        marker_array.markers[i].ns = "marker";
        marker_array.markers[i].id = i;

        marker_array.markers[i].type = visualization_msgs::Marker::CYLINDER;
        marker_array.markers[i].action = visualization_msgs::Marker::ADD;

        marker_array.markers[i].pose.position.x =  map.at(i).x;
        marker_array.markers[i].pose.position.y =  map.at(i).y;
        marker_array.markers[i].pose.position.z = 0;

        marker_array.markers[i].pose.orientation.x = 0.0;
        marker_array.markers[i].pose.orientation.y = 0.0;
        marker_array.markers[i].pose.orientation.z = 0.0;
        marker_array.markers[i].pose.orientation.w = 1.0;

        marker_array.markers[i].scale.x = 0.072;
        marker_array.markers[i].scale.y = 0.072;
        marker_array.markers[i].scale.z = 1;

        marker_array.markers[i].color.r = 0.0f;
        marker_array.markers[i].color.g = 0.0f;
        marker_array.markers[i].color.b = 1.0f;
        marker_array.markers[i].color.a = 1.0f;
      
      }
      marker_pub.publish(marker_array);


        rate.sleep();
    }

  return 0;
}
