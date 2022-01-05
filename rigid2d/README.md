# Rigid2D Package
A C++ library for representing 2D rigid body transformations.The main purpose of this library is to convert the rigid2d library into a ROS package.It also includes library for  Kinematics of wheeled mobile robots.


# Rigid2D Library 

### rigid2d.cpp  
This implements the rigid body 2D transformations  which includes Transformation2D ,Vector2D and Twist 2D

### diff_drive.cpp  
This provides a class for modeling a differential drive robot. It has fields for tracking the robot's current  pose and updating its speed and velocity.

# ROS Nodes 

### fake_turtle.cpp 
This node subscribers to turtlesim_teleop_key  and publishes encoder messgea to /joint_states so that we can run the robot in RVIZ without setting up the real robot.

### odometer_node.cpp
This node provides tracking and updating of a robot's odometry from published encoder data. 


# Launch file 

### fake_turtle_odom.launch
lets you visualize and control the turtlebot on rviz
`export TURTLEBOT3_MODEL=burger`
`roslaunch rigid2d fake_turtle_odom.launch `


![Demonstration](https://github.com/ME495-Navigation/assignment-ashleetiw/blob/main/rigid2d/diff_drive.gif)