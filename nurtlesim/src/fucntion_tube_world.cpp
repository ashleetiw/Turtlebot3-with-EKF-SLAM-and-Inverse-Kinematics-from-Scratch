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
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include<sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include<string>
#include <iterator>
#include <random>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>

// GLOBAL VARS
std::string odom_id, body_id, left_wheel, right_wheel;
double wheel_base, wheel_radius;
rigid2d::Pose pose,pose_true;
rigid2d::DiffDrive driver,d_true;
rigid2d::Twist2D Vb,Vb_true;      // body cmd_vel
double frequency = 100;
std::vector<double> array;
bool flag= false;

std::mt19937_64 &getrandom()
{
  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  return gen;
}

double NormalDistribution(const double mu, const double sigma)
{
  std::normal_distribution<double> dis(mu, sigma);
  return dis(getrandom());
}


void vel_callback(const geometry_msgs::Twist &tw)
{ 
  /// \brief cmd_vel subscriber callback. Records commanded twist
  ///
  /// \param tw (geometry_msgs::Twist): the commanded linear and angular velocity
  /// \returns wheel (rigid2d::wheel_vel): the left and right wheel angles
  const double mean = 0.1;
  const double var = 0.1;
  Vb.angle_z=tw.angular.z/frequency +NormalDistribution(mean, var);
  Vb.linear_vx=tw.linear.x/frequency+NormalDistribution(mean, var);
  Vb.linear_vy=tw.linear.y/frequency+NormalDistribution(mean, var);
  driver.forward(Vb,wheel_radius,wheel_base);

  Vb_true.angle_z=tw.angular.z/frequency;
  Vb_true.linear_vx=tw.linear.x/frequency;
  Vb_true.linear_vy=tw.linear.y/frequency;

  d_true.forward(Vb_true,wheel_radius,wheel_base);

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
  nh.getParam("location",array);
  nh.getParam("odom_frame_id", odom_id);
  nh.getParam("body_frame_id", body_id);



  // Init Subscriber
  ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 1, vel_callback);
  // Init Publisher
  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  bool latch = true;
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("marker", 100,latch);
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("real_path", 10);
  ros::Publisher marker_pub_fake = nh.advertise<visualization_msgs::MarkerArray>("fake_sensor", 10);
  ros::Publisher marker_pub_robot = nh.advertise<visualization_msgs::MarkerArray>("robot_marker", 100);

  // init pose to/
  // Init Time
  ros::Time current_time;
  current_time = ros::Time::now();

  ros::Rate rate(frequency);

  visualization_msgs::MarkerArray marker_array,marker_array2,marker_arrayr;
  tf2_ros::TransformBroadcaster world_odom_broadcaster;
  tf2_ros::TransformBroadcaster world_turtle_broadcaster;
  nav_msgs::Path path;
  geometry_msgs::PoseStamped path_pose;  
  int count=0;


   // robot model
  driver= rigid2d::DiffDrive(pose,wheel_radius,wheel_base);
  // Main Whil
  while (ros::ok())
  {
  	ros::spinOnce();
    current_time = ros::Time::now();

    pose=driver.getpose();
    
    pose_true=d_true.getpose();

    sensor_msgs::JointState joint;

    joint.header.stamp = current_time;

    rigid2d::Encoders encoders = driver.getEncoders();
      // joint stores vectors, so we push back the name corresp. to left wheel joint
      joint.name.push_back(left_wheel);
      // then we insert the left wheel encoder value
      joint.position.push_back(encoders.leftangle);
      // repeat with right wheel. Note order must be consistent between name pushback and
      // encoder value pushback
      joint.name.push_back(right_wheel);
      joint.position.push_back(encoders.rightangle);
      joint_pub.publish(joint);


      // broadcast transform between world and turtle
      geometry_msgs::TransformStamped tf_mo;
      tf_mo.header.stamp = ros::Time::now();
      tf_mo.header.frame_id ="world";
      tf_mo.child_frame_id = "turtle";

      tf2::Quaternion q_mo;
      q_mo.setRPY(0, 0, pose.theta-pose_true.theta);
      geometry_msgs::Quaternion quat_mo;
      quat_mo = tf2::toMsg(q_mo);

      tf_mo.transform.translation.x =pose.x-pose_true.x;
      tf_mo.transform.translation.y = pose.y-pose_true.y;
      tf_mo.transform.translation.z = 0.0;
      tf_mo.transform.rotation = quat_mo;
      world_turtle_broadcaster.sendTransform(tf_mo);

      // world to odom such that world to base link in the robot pose 
      // d_true= d_true.get_pose();
      // Construct Tmb
      rigid2d::Vector2D Vmb = rigid2d::Vector2D(pose.x, pose.y);
      rigid2d::Transform2D Tmb = rigid2d::Transform2D(Vmb, pose.theta);
      // Construct Tob
      rigid2d::Vector2D Vob = rigid2d::Vector2D(pose_true.x, pose_true.y);
      rigid2d::Transform2D Tob = rigid2d::Transform2D(Vob, pose_true.theta);
      // Now find Tmo
      rigid2d::Transform2D Tmo = Tmb * Tob.inv();
  
      // geometry_msgs::TransformStamped odom_tf;
      // odom_tf.header.stamp = ros::Time::now();
      // odom_tf.header.frame_id = "world";
      // odom_tf.child_frame_id = "odom";
      // // Pose
      // odom_tf.transform.translation.x = Tmo.dx;
      // odom_tf.transform.translation.y = Tmo.dy;
      // odom_tf.transform.translation.z = 0;
      // // use tf2 to create transform
      // tf2::Quaternion q;
      // q.setRPY(0, 0, Tmo.angle);
      // geometry_msgs::Quaternion odom_quat = tf2::toMsg(q);
      // odom_tf.transform.rotation = odom_quat;
      // // Send the Transform
      // world_odom_broadcaster.sendTransform(odom_tf);

  //       // robot model in ground_truth frame

      /*********************    robot marker *************************/

      marker_arrayr.markers.resize(1);
      marker_arrayr.markers[0].header.frame_id ="world"; 
      marker_arrayr.markers[0].header.stamp = ros::Time::now();
      marker_arrayr.markers[0].lifetime = ros::Duration(5.0); 
      marker_arrayr.markers[0].ns = "robot";
      marker_arrayr.markers[0].id = 1;

      marker_arrayr.markers[0].type = visualization_msgs::Marker::CYLINDER;
      marker_arrayr.markers[0].action = visualization_msgs::Marker::ADD;

      marker_arrayr.markers[0].pose.position.x = pose.x;
      marker_arrayr.markers[0].pose.position.y = pose.y;
      marker_arrayr.markers[0].pose.position.z = 0.0;

      marker_arrayr.markers[0].pose.orientation.x = 0.0;
      marker_arrayr.markers[0].pose.orientation.y = 0.0;
      marker_arrayr.markers[0].pose.orientation.z = 0.0;
      marker_arrayr.markers[0].pose.orientation.w = 1.0;

      marker_arrayr.markers[0].scale.x = 0.5 ;
      marker_arrayr.markers[0].scale.y = 0.5 ;
      marker_arrayr.markers[0].scale.z = 0;

      marker_arrayr.markers[0].color.r = 0.0f;
      marker_arrayr.markers[0].color.g = 0.0f;
      marker_arrayr.markers[0].color.b = 1.0f;
      marker_arrayr.markers[0].color.a = 1.0f;

      marker_pub_robot.publish(marker_arrayr);

  //     // ROS_ERROR_STREAM("after gt marker");
  //     //       //\//////////////////////////////////////////  ground truth markers  ///////////////////////////

  marker_array.markers.resize(array.size());
  // // ROS_ERROR_STREAM(array.size());
  unsigned int n=array.size();
  for(unsigned int i = 0;i<n;i++)  {
              // ROS_ERROR_STREAM(i);
              marker_array.markers[i].header.frame_id ="world";
              marker_array.markers[i].header.stamp = ros::Time::now();
              marker_array.markers[i].lifetime = ros::Duration(1); 
              marker_array.markers[i].ns = "real";
              marker_array.markers[i].id = i;

              marker_array.markers[i].type = visualization_msgs::Marker::CYLINDER;
              marker_array.markers[i].action = visualization_msgs::Marker::ADD;

              marker_array.markers[i].pose.position.x = array[i];
              marker_array.markers[i].pose.position.y =array[i+1];
              marker_array.markers[i].pose.position.z = 0.0;

              marker_array.markers[i].pose.orientation.x = 0.0;
              marker_array.markers[i].pose.orientation.y = 0.0;
              marker_array.markers[i].pose.orientation.z = 0.0;
              marker_array.markers[i].pose.orientation.w = 1.0;

              marker_array.markers[i].scale.x = 1;
              marker_array.markers[i].scale.y = 1;
              marker_array.markers[i].scale.z = 2.0;

              marker_array.markers[i].color.r = 0.0f;
              marker_array.markers[i].color.g = 1.0f;
              marker_array.markers[i].color.b = 0.0f;
              marker_array.markers[i].color.a = 1.0f;

            }

      marker_pub.publish(marker_array);
    
  //     // // /////////////////////////  turtle frame - markers //////////////////////////////////
      double noise=0;


      n=array.size();
      marker_array2.markers.resize(array.size());
      
      for(unsigned int j = 0;j<n;j++)
          {  
              //  4 is currently the maximum range 
             
                    marker_array2.markers[j].header.frame_id ="world"; 
                    marker_array2.markers[j].header.stamp = ros::Time::now();
                    marker_array2.markers[j].lifetime = ros::Duration(5.0); 
                    marker_array2.markers[j].ns = "fake";
                    marker_array2.markers[j].id = j;

                    marker_array2.markers[j].type = visualization_msgs::Marker::CYLINDER;
                    marker_array2.markers[j].action = visualization_msgs::Marker::ADD;

                    marker_array2.markers[j].pose.position.x =array[j]-pose.x+noise;
                    marker_array2.markers[j].pose.position.y=array[j+1]-pose.y+noise;
                    marker_array2.markers[j].pose.position.z = 0.0;

                    marker_array2.markers[j].pose.orientation.x = 0.0;
                    marker_array2.markers[j].pose.orientation.y = 0.0;
                    marker_array2.markers[j].pose.orientation.z = 0.0;
                    marker_array2.markers[j].pose.orientation.w = 1.0;

                    marker_array2.markers[j].scale.x = 1.0 ;
                    marker_array2.markers[j].scale.y = 1.0 ;
                    marker_array2.markers[j].scale.z = 2.0;

                    marker_array2.markers[j].color.r = 1.0f;
                    marker_array2.markers[j].color.g = 0.0f;
                    marker_array2.markers[j].color.b = 0.0f;
                    marker_array2.markers[j].color.a = 1.0f;
                    //  if ( (pow((array[j]-driver.x),2)+ pow((array[j+1]-driver.y),2))<4 ){


      }

    // maintaing 10 HZ freq
      if (count%10==0){
        // ROS_ERROR_STREAM(count);
      marker_pub_fake.publish(marker_array2);
      }

      path_pose.pose.position.x = pose_true.x;
      path_pose.pose.position.y = pose_true.y;
      path_pose.header.stamp = ros::Time::now();
      path_pose.header.frame_id = "world";

      path.poses.push_back(path_pose);
      path.header.frame_id = "world";
      path.header.stamp = ros::Time::now();
      path_pub.publish(path);


      count=count+1;



      rate.sleep();

  }

  return 0;
}