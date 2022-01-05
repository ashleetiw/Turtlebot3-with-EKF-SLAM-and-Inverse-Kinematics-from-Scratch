/// \file   lets the robot drive in a circle of a specified radius at a specified speed
/// \brief
///
/// PUBLISHES:
///   cmd_vel (geometry_msgs/Twist): Commanded body twist
///
/// SERVICES:
///   control (nuturtle_robot/control) - sets the rotation direction of the robot
#include <sstream>
#include <stdlib.h>
#include <cmath> 
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nuturtle_robot/control.h"

class circle{
    private:
        ros::NodeHandle n;    
        ros::Publisher pub;
        ros::ServiceServer service;

    public:

        double speed,radius,state;
      
        circle(){
            /// PARAMETERS:
            /// radius : radius of the circle to move around 
            /// speed: speed to move in a circular trajectory
            n.getParam("radius",radius);
            n.getParam("speed",speed);
            

            pub=n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
            service=n.advertiseService("control", &circle::service_callback,this);

        }

        bool service_callback(nuturtle_robot::control::Request &req ,nuturtle_robot::control::Response &resp){

        /// \brief service sets the rotation direction of the robot
        /// \param req - service request direction
        /// \param res - service direction result
         /// \returns true if service is completed
            state=req.state;

       
            ros::Rate rate(50);
            geometry_msgs::Twist t;
            t.linear.x=0;
            t.linear.y=0;
            while(ros::ok()){
                if (req.state==0){
                /*stop serivce*/
                t.angular.z=0;

                }
                else if (req.state==1){
                    /*clockwise service*/
                    t.linear.x=speed*radius;
                    t.angular.z=speed;
                }
                else if(req.state==2){
                    /* anti clockwise service*/
                    t.linear.x=-speed*radius;
                    t.angular.z=-speed;

                }
                pub.publish(t);
                rate.sleep();
                

            }
            return true;
        }


};


/// \brief initialises turtle_interface node 
int main(int argc,char **argv){
   
    ros::init(argc, argv, "follow_circle");

    circle t;  
    
    ros::spin();
    return 0;
}