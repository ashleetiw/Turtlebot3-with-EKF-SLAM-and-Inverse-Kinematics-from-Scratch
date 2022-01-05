# Turtlebot3 with EKF-SLAM and Inverse Kinematics from Scratch

## Demo
![s](https://github.com/ashleetiw/Turtlebot3-with-EKF-SLAM-and-Inverse-Kinematics-from-Scratch/blob/master/simulation.gif)
![r](https://github.com/ashleetiw/Turtlebot3-with-EKF-SLAM-and-Inverse-Kinematics-from-Scratch/blob/master/realworld.gif)


The steps can be summarized as the following:

1. The turtlebot laser scan first need to be programmed to detect landmarks and wall shapes
2. Each data point will be grouped into different landmarks or walls using clustering
3. Perform circle regression to extract the landmarks location based on each cluster of laser scan points
4. Classify the clusters of points into wall or cylinder landmark
5. Associating data to landmarks using mahalanobis distance
6. Perform odometry calculation
7. Update EKF-SLAM using associated landmark position and odometry information

# ROS Package 
This repository consists of several ROS packages
- `nuturtle_description`  -  package contains descriptions of the turtlebot3 model's geometry and urdf
- `nuturtle_robot` -Interfaces with TurtleBot3 hardware and contains launch files to follow circle
- `rigid2d` - package contains 2D transformations, vectors, twists, model for  differential drive robot and nodes for modeling encoders readings and odometry
- `turtle_rect` - moves the turtle in a rectangular trajectory.
- ` nurtlesim` - to create a simulated environment which provides sensor data
- ` nuslam` - to perfrom SLAM

# DEPENDCIES
- `nuturtlebot` -additional package to work with the lower-level hardware on the turtlebot
- `catch_ros` -Using the package to integrate your unit tests with ROS
- `turtlebot3`- to launhc turtlsim teleop key node
- `armadillo library` - to perform matrix calculations
