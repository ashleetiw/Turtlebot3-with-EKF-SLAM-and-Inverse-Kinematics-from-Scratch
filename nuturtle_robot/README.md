# Nurturtle_robot pacakge 
The package nterfaces with TurtleBot3 hardware and contains launch files

## Launch files

## basic_remote.launch
`roslaunch nuturtle_robot basic_remote.launch` 
The launchfil lets you run nodes on the turtlebot from your computer.
The launchfile loads serial_node.py which communications with the OpenCR board to give you low-level control through ROS


### Configuration Options
Argument:`robot`which allows the user to specify the hostname of the turtlebot.
- Add the `robot:=localhost` or  `robot:=turtlebot` to choose the host computer
- Add the `circle:=true` option to follow circle and `circle:=false`to control using turtlebot3_teleop_key_nodes

## odom_teleop.launch
Launches either the follow_circle or turtlebot3_teleop_key nodes
Launches rviz in a mode that lets you visualize the position of the robot in the odometry frame
Case 1:Launches the odometer and turtle_interface nodes
`roslaunch nuturtle_robot odom_teleop.launch robot:=turtlebot circle:=false`

Case 2:Launches follow_cicle  node
`roslaunch nuturtle_robot odom_teleop.launch robot:=turtlebot circle:=true`
`rosservice call \control 1` if rotate clockwise 
`rosservice call \control -1` if rotate anti-clockwise 


# ROS Nodes 

### turtle_interface.cpp 
The node implements our own low-level control and sensor routines in ROS

### follow_circle.cpp 
The node publishes commands that let the robot drive in a circle of a specified radius at a specified speed


# Services 
### control 
The service causes the robot to travel either clockwise, counter clockwise, or stop



# Answers after experimentation 

After Driving the robot forward and backward in a straight line several times, and then stopping its configuration is (-0.87,0.34,0)
![Demonstration](https://github.com/ME495-Navigation/assignment-ashleetiw/blob/main/nuturtle_robot/forward_turtlebot.gif)
After Rotating the robot clockwise and counter clockwise several times, stopping when the turtlebot its configuration is (-1.32,-1.26,0)
![Demonstration](https://github.com/ME495-Navigation/assignment-ashleetiw/blob/main/nuturtle_robot/rotate_turtlebot.gif)

Driving the robot in a circle, clockwise and counter clockwise several times, stopping when the turtlebot is its  near initial configuration.The final coordinate is (-0.453,0.2,0)


I experiemented driving robot in carpet , wooden floor and on window sill .When tried moving forward and backward several times , the best result was on window sill and the worst result on carpet

I also experimented with the speed of the turtlebot while rotating .The higher the speed was the far it stopped from initial configuration 

