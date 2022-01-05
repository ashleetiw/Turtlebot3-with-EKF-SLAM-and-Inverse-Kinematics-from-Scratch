#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP 
#define DIFF_DRIVE_INCLUDE_GUARD_HPP


/// \file
/// \brief Library for two-dimensional rigid body for a diff drive robot.

#include "rigid2d/rigid2d.hpp"

namespace rigid2d{

      /// \brief pose for a 2D diff drive robot
      struct Pose
      {
        double theta = 0.0; /*!< direction in radians. */
        double x = 0.0; /*!< x axis position. */
        double y = 0.0; /*!< y axis position. */
      };

      /// \brief keep the angular velocity of each wheel
      struct WheelVel
      {
        double wl = 0.0; 
        double wr = 0.0; 
      };
 
      struct Encoders
      {
        double leftangle = 0.0; 
        double rightangle = 0.0; 
      };

///////////////////////////////////////////////////////////////////////////

    /// \brief differential class for robots
    class DiffDrive
    {
    public:
        /// \brief Default constructor for DiffDrive model 
        DiffDrive();

        /// \brief create a DiffDrive object by specifying the position, and geometry(distance wheel, wheel radious)
        /// \param pose - the current position of the robot
        /// \param wheel_base - the distance between the wheel centers
        /// \param wheel_radius - the raidus of the wheels
        DiffDrive(Pose pose, double wheel_radius,double wheel_length);

        /// \brief Convert Twist velocity to wheel velocity
        /// \param twist - the robots velocity      
        /// \returns - the wheel velocities to use
        /// \throws std::exception
        WheelVel twistToWheels(Twist2D t,double wheel_radius,double wheel_length);

        /// \brief Convert wheel velocity velocity to Twist
        /// \param vel - the robots wheel velocity      
        /// \returns - the wheel velocities of the robot
        Twist2D wheelsToTwist(WheelVel w,double wheel_radius,double wheel_length) ;

        /// \brief Update the robot's odometry based on the current encoder readings
        /// \param left - the left encoder angle (in radians)
        /// \param right - the right encoder angle (in radians)
        /// \return the velocity of each wheel
        WheelVel update_Odom(double left, double right,double wheel_radius,double wheel_length);


        /// \brief update the odometry of the diff drive robot
        /// \param vel - the twist velocity to apply at robot
        void forward(Twist2D &vel,double wheel_radius,double wheel_length);

        /// \brief get the current pose of the robot
        Pose getpose();

        /// \brief get the wheel speeds, based on the last encoder update
        WheelVel wheelvel();

        /// \brief set the position of robot
        /// \param ps -Pose give the position/orientation
        Pose setpose(Pose pose);

        /// \brief get the current wheel encoder readings
        /// \return the angular position of wheels
        Encoders getEncoders();
        Encoders setEncoders(Encoders enc);

        private:
        Pose pose; // theta, x, y of robots pose
        Encoders encoders; // left and right current encoder readings
        WheelVel wheelVel; // the current wheel velocities
    };

}


#endif
