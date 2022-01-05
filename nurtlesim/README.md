# Package: nurtlseim

## Package Summary
This incorporates creating a simulated environment called tube world
Part1 :Publishes noise tube data as sensor data

Part2: Publsihes lidar scan data as sensor data 

## Launch Instructions

Run `roslaunch nuslam <landmark_detection.launch> 

Run `roslaunch nurtulism tube_world.launch` to launch both types of sensor data 

## tube.cpp
The node provides noisy sensor data 
Note : to independently run the node transforamtion from world to odom needs to be uncommented 


Sensor data from simulator 
![Demonstration](https://github.com/ME495-Navigation/assignment-ashleetiw/blob/main/nurtlesim/sensordata.png?raw=true)


Sensor data from LIDAR
![Demonstration](https://github.com/ME495-Navigation/assignment-ashleetiw/blob/main/nurtlesim/laser2.png?raw=true)


