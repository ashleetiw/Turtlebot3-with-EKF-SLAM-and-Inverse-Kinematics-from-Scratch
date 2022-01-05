/// \file
/// \brief the code that interacts with the turtlebot hardware

/// PUBLISHES:
/// joint_states (sensor_msgs/JointState): the turtlebot3's wheel positions and velocities
/// wheel_cmd (nuturtlebot/WheelCommands): the turtlebot3's wheel commands; integers corresponding to wheel velocities from -max to max in rad/s
///
/// SUBSCRIBES:
/// cmd_vel (geometry_msgs/Twist): subscriber, which records the commanded twist
/// sensor_data (nuturtlebot/SensorData): subscriber, which records wheel encoder values, among other turtlebot3 sensor data
///

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include<sensor_msgs/JointState.h>
#include<sensor_msgs/JointState.h>
#include<string>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "nuturtlebot/SensorData.h"
#include "nuturtlebot/WheelCommands.h"

class nuturtle{

    private:
        ros::NodeHandle n;    
        ros::Subscriber sub_vel;
        ros::Subscriber sub_sensor;
        ros::Publisher pub_wheel;
        ros::Publisher pub_joint;

    public:
        /// variables declared
        std::string right_wheel,left_wheel;
        float max_linear_vel = 0.22;
        float max_angular_vel = 2.84;
        float motor_rot_max_ = 6.35492;
        float encoder_ticks = 4096;
        double frequency=50;
        double wheel_base,wheel_radius,wheel_right_angle,wheel_left_angle;
        bool vel_flag = false;
        bool sensor_flag=false;
        rigid2d::WheelVel wheelVel,wheel_angle;

        rigid2d::DiffDrive d;
        nuturtle(){
            /// \brief initialises the required publisher and subscriber
            /// PARAMETERS:
            ///  (string)left_wheel_joint: the left wheel joint name 
            ///   (string)right_wheel_joint: the right wheel joint name 
            ///   (double)wheel_base: distance between two wheels 
            ///   (double)wheel_radius: wheel radius
            ///   (double)frequency: frequency of while loop
            n.getParam("left_wheel_joint",left_wheel);
            n.getParam("right_wheel_joint",right_wheel);
            n.getParam("wheel_base",wheel_base);
            n.getParam("wheel_radius",wheel_radius);
           
            ROS_ERROR_STREAM('reaching here');

            sub_vel = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1,&nuturtle::callback_vel,this);
            sub_sensor = n.subscribe<nuturtlebot::SensorData>("sensor_data", 1,&nuturtle::callback_sensor,this);

            pub_joint= n.advertise<sensor_msgs::JointState>("joint_states", 1);
            pub_wheel =n.advertise<nuturtlebot::WheelCommands>("wheel_cmd", 1);


                        // Init Time
            ros::Time current_time;
            current_time = ros::Time::now();

            ros::Rate rate(frequency);
            // Main While
            while (ros::ok())
            {
                ros::spinOnce();
                current_time = ros::Time::now();

                if (sensor_flag == true)
                {
                sensor_msgs::JointState joint;

                joint.header.stamp = current_time;

                // joint stores vectors, so we push back the name corresp. to left wheel joint
                joint.name.push_back(right_wheel);
                // then we insert the left wheel encoder value
                joint.position.push_back(wheel_angle.wl);
                joint.velocity.push_back(wheelVel.wl);

                // repeat with right wheel. Note order must be consistent between name pushback and
                // encoder value pushback
                joint.name.push_back(left_wheel);
                joint.position.push_back(wheel_angle.wr);
                joint.velocity.push_back(wheelVel.wr);

                // now publish
                // std::cout << wheel_angle.ul << std::endl;
                // std::cout << wheel_angle.ur << std::endl;
                pub_joint.publish(joint);

                sensor_flag = false;
                }

                if (vel_flag == true)
                {
                nuturtlebot::WheelCommands wc;

                wc.left_velocity = std::round(wheelVel.wl);
                wc.right_velocity = std::round(wheelVel.wr);

                pub_wheel.publish(wc);

                vel_flag = false;
                }
                rate.sleep();

            }

        
        }


        void callback_vel(const geometry_msgs::Twist twist) {
            /// \brief cmd_vel subscriber callback that will make the turtlebot3 follow the specified twist
            ///
            /// \param t (geometry_msgs/Twist): gives linear and angular velocity
            /// \ publishes  wheel velocites (rigid2d::WheelVelocities --> nuturtlebot:WheelCommands)

            ROS_ERROR_STREAM('reaching in callback');
            float ang_vel = twist.angular.z;
            if (ang_vel > max_angular_vel)
            {
                ang_vel = max_angular_vel;
            } else if (ang_vel < - max_angular_vel)
            {
                ang_vel = -max_angular_vel;
            }
            // Cap Linear Twist
            float lin_vel = twist.linear.x;
            if (lin_vel > max_linear_vel)
            {
                lin_vel = max_linear_vel;
            } else if (lin_vel < - max_linear_vel)
            {
                lin_vel = -max_linear_vel;
            }

            rigid2d::Twist2D Vb;
            Vb.angle_z=ang_vel;
            Vb.linear_vx=lin_vel;
            Vb.linear_vy=twist.linear.y;
            // Get Wheel Velocities

            wheelVel = d.twistToWheels(Vb,wheel_radius,wheel_base);

            // Cap Wheel Velocities
            if (wheelVel.wl > motor_rot_max_)
            {
                wheelVel.wl = motor_rot_max_;
            } else if (wheelVel.wl < - motor_rot_max_)
            {
                wheelVel.wl = - motor_rot_max_;
            }

            if (wheelVel.wr > motor_rot_max_)
            {
                wheelVel.wr = motor_rot_max_;
            } else if (wheelVel.wr < - motor_rot_max_)
            {
                wheelVel.wr = - motor_rot_max_;
            }

            // Now convert wheel vel to integers from -256 to 256
            float m = (256. - - 256.) / (motor_rot_max_ * 2.);
            float b = (256. - motor_rot_max_ * m);

            wheelVel.wl = wheelVel.wl * m + b;
            wheelVel.wr = wheelVel.wr * m + b;

            vel_flag = true;

        }

        void callback_sensor(const nuturtlebot::SensorData sns){
        /// \brief sensor_data subscriber callback. Records left and right wheel angles
        ///
        /// \param sns (nuturtlebot/SensorData ): the left and right wheel joint sensor data 
        /// wheel_angle and wheelVel_measured (rigid2d::WheelVelocities): measured wheel angles and velocities respct.
           

            wheel_angle.wl = sns.left_encoder;
            wheel_angle.wr= sns.right_encoder;

            // Now convert encoder values (0-4096 to 0-2pi)
            float m = (2.0 * rigid2d::PI) / (encoder_ticks);
            float b = (2.0 * rigid2d::PI - encoder_ticks * m);

            wheel_angle.wl = wheel_angle.wl * m + b; 
            wheel_angle.wr = wheel_angle.wr * m + b;

            // Normalize Encoder Values
            wheel_angle.wl = rigid2d::normalize_angle(wheel_angle.wl);
            wheel_angle.wr= rigid2d::normalize_angle(wheel_angle.wr);

            // Get wheel velocities based on encoder data
            wheelVel= d.update_Odom( wheel_angle.wl,wheel_angle.wr,wheel_radius,wheel_base);

            sensor_flag = true;

        }

};


int main(int argc,char **argv){
   /// \brief initialises turtle_interface node 
    ros::init(argc, argv, "turtle_interface");
    ROS_ERROR_STREAM(" calling main");
    nuturtle t;  
    
  
    // ,wheel_angle;
    // rigid2d::DiffDrive d;
    
    ros::spin();
    return 0;
}