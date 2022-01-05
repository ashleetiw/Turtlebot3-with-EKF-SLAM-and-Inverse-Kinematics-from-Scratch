# Package: nuslam

## Package Summary

This incorporates feature detection on the Turtlebot3's LiDAR, as well as EKF SLAM with Unknown Data Association using the Mahalanobis Distance.


Feature Detection:  blue is the landmarks detected

![Demonstration](https://github.com/ME495-Navigation/assignment-ashleetiw/blob/main/nuslam/landmark_detection.png?raw=true)

## Launch Instructions


Run `roslaunch nuslam landmarks.launch` to launch the feature detection node.

Run `roslaunch nuslam slam.launch ` to launch the EKF SLAM node + turtlebot3_teleop

## landmarks.cpp

Subcribers to `scan ` data for feature detection and publishes `markers` with the corresponding location in world frame.
For landmark detection from clusters 
* Eliminated circles with really large or small radii,
* Implemented circle fitting 
* Classifed the clusters of points into Circle and Not Circle

## circle_test.cpp
Tests for the following algorithms


## kalman.hpp/cpp
Kalman filter implmentation

## slam.cpp
Contains the node implementation of SLAM