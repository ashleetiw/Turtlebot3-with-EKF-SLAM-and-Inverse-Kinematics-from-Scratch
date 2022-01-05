#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"
#include <iostream>
#include <math.h>

rigid2d::DiffDrive::DiffDrive(){
        pose = Pose{0,0,0};
        wheelVel = {0,0};
        encoders = {0,0};
}

rigid2d::DiffDrive::DiffDrive(Pose pose,double wheel_radius,double wheel_length){
        pose = pose;
      
        // initialize Velocity and encoders to ZERO
        wheelVel = {0,0};
        encoders ={0,0};
}

rigid2d::Twist2D rigid2d::DiffDrive::wheelsToTwist(rigid2d::WheelVel w,double wheel_radius,double wheel_length){
        rigid2d::Twist2D tnew;
        tnew.linear_vy=0;
        tnew.linear_vx=((w.wr+w.wl)*wheel_radius)/2;    /* w average *r */
        tnew.angle_z=((w.wr-w.wl)*wheel_radius)/wheel_length;
        return tnew;
}


rigid2d::WheelVel rigid2d::DiffDrive::twistToWheels(rigid2d::Twist2D t,double wheel_radius,double wheel_length){
    rigid2d::WheelVel w;
 
    double omega=t.angle_z;
    double vx=t.linear_vx;
    double d = wheel_length / 2;
 
    w.wr=(1 / wheel_radius) * (d*omega + vx);
    w.wl=(1 / wheel_radius) * (-d*omega + vx);
  
    return w; 
}

rigid2d::Pose rigid2d::DiffDrive::getpose(){
        return pose;
    }

rigid2d::Pose rigid2d::DiffDrive::setpose(rigid2d::Pose pose){
    pose = pose;
    pose.theta = rigid2d::normalize_angle(pose.theta);
    return pose;
}

rigid2d::Encoders rigid2d::DiffDrive::getEncoders(){
    return encoders;
}

rigid2d::Encoders rigid2d::DiffDrive::setEncoders(rigid2d::Encoders enc){
    encoders.rightangle = normalize_angle(enc.rightangle);
    encoders.leftangle = normalize_angle(enc.leftangle);

    return encoders;
}


rigid2d::WheelVel rigid2d::DiffDrive::wheelvel(){
       return wheelVel;
    }

rigid2d::WheelVel rigid2d::DiffDrive::update_Odom(double left, double right,double wheel_radius,double wheel_length){
    // update wheel velocity
        wheelVel.wl = normalize_angle(left - encoders.leftangle);
        wheelVel.wr = normalize_angle(right - encoders.rightangle);

        //update encoders 
        encoders.leftangle = normalize_angle(left);
        encoders.rightangle = normalize_angle(right);

        // go from wheels velocity to twist (robots velocity)
        rigid2d::Twist2D vel = wheelsToTwist(wheelVel,wheel_radius,wheel_length);
        rigid2d::Vector2D xy_pose;
        xy_pose.x=pose.x;
        xy_pose.y=pose.y;
        rigid2d::Transform2D tf_r;
        tf_r.dx=xy_pose.x;
        tf_r.dy=xy_pose.y;
         tf_r.angle=pose.theta;
        tf_r=tf_r.integrateTwist(vel);

        //update position
        pose.x = tf_r.dx;
        pose.y = tf_r.dy;
        pose.theta = tf_r.angle;

        return wheelVel;

    }

void rigid2d::DiffDrive::forward(Twist2D &vel,double wheel_radius,double wheel_length){
        // update wheel velocity and encoders + normalize 
        wheelVel = twistToWheels(vel,wheel_radius,wheel_length);
        wheelVel.wl = normalize_angle(wheelVel.wl);
        wheelVel.wr = normalize_angle(wheelVel.wr);
        encoders.leftangle = normalize_angle(encoders.leftangle+wheelVel.wl);
        encoders.rightangle = normalize_angle(encoders.rightangle+wheelVel.wr);
       
        rigid2d::Vector2D xy_pose;
        xy_pose.x=pose.x;
        xy_pose.y=pose.y;
        rigid2d::Transform2D tf_r;
        tf_r.dx=xy_pose.x;
        tf_r.dy=xy_pose.y;
         tf_r.angle=pose.theta;
        tf_r=tf_r.integrateTwist(vel);
        //update position
        pose.x = tf_r.dx;
        pose.y = tf_r.dy;
        pose.theta = tf_r.angle;
     

    }