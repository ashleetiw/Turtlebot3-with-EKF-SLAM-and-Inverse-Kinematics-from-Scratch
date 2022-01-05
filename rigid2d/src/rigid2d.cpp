/**************************************************  implementation file ************************************************/
#include "rigid2d/rigid2d.hpp"
#include <iostream>
#include <math.h>


double rigid2d::normalize_angle(double rad){
        double PI=3.14159265358979323846;
        double p  = std::floor( (rad + PI) / (2.0 * PI) );
      		rad = (rad + PI) - p * 2.0 * PI;

      		if (rad < 0) {
        		rad = rad + 2.0 * PI;
      		}		
	
      		return rad - PI;
	}

std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Vector2D & v){
        os << v.x;
        os << v.y;
        return os;
}
    

std::istream & rigid2d::operator>>(std::istream & is,rigid2d::Vector2D & v){ 
        is >> v.x;
        is >> v.y;
        return is;}

rigid2d::Vector2D::Vector2D(){
        x=0;
        y=0;
}

rigid2d::Vector2D::Vector2D(double x,double y){
        x=x;
        y=y;
}

rigid2d::Vector2D rigid2d::Vector2D::normalize(const rigid2d::Vector2D & v){ 

        rigid2d::Vector2D vnew;
        double val;
        val=sqrt(pow(v.x,2)+pow(v.y,2));


        if(val==0){ /* case of 0 vector  */
             vnew.x=0;
             vnew.y=0;   
        }
        else{
        vnew.x=v.x/val;
        vnew.y=v.y/val;
        }
        return vnew;
}

rigid2d::Vector2D & rigid2d::Vector2D::operator+=(const rigid2d::Vector2D & v){
        x=x+v.x;
        y=y+v.y;

       return *this;
        
}

rigid2d::Vector2D & rigid2d::Vector2D::operator-=(const rigid2d::Vector2D & v){
        x=x-v.x;
        y=y-v.y;

       return *this;
        
}

rigid2d::Vector2D & rigid2d::Vector2D::operator*=(const double s){
        x=x*s;
        y=y*s;

       return *this;
        
}


rigid2d::Vector2D & rigid2d::operator+(const rigid2d::Vector2D & lhs, rigid2d::Vector2D & rhs){
                // rhs.x=rhs.x+lhs.x;
                // rhs.y=rhs.y+lhs.y;
                // return rhs;
                rhs.operator+=(lhs);
                return rhs;
               
}

rigid2d::Vector2D & rigid2d::operator-(const rigid2d::Vector2D & lhs, rigid2d::Vector2D & rhs){
                // rhs.x=rhs.x-lhs.x;
                // rhs.y=rhs.y-lhs.y;
                // return rhs;
                rhs-=lhs;
                return rhs;
}


rigid2d::Vector2D & rigid2d::operator*(rigid2d::Vector2D & lhs, const double s){
        // lhs=lhs*s;
        // lhs=lhs*s;
        lhs*=s;

       return lhs;
        
}

double rigid2d::Vector2D::magnitude(){ 

        double val;
        val=sqrt(pow(this->x,2)+pow(this->y,2));
        return val;
}

double rigid2d::Vector2D::angle()
{
        double angle;
	angle=atan(this->y /this->x);
        return angle;
}



rigid2d::Transform2D::Transform2D()   /* initlaised with identity matrix */
{   
    angle = 0;/* cos =1 sin=0 hence gives identity matrix  */
    dx = 0;
    dy = 0;   
}

rigid2d::Transform2D::Transform2D(const Vector2D & trans)
{	
	/* Pure Translational Transform
          [1 0 dx
          0 1 dy
          0 0 1 ]  */
	dx=trans.x;
        dy=trans.y;
        angle=0; /* no rotation */ 
      

}
       
      
rigid2d::Transform2D::Transform2D(double radians){
        /* Pure Rotational Transform
        [ cos -sin 0
         sin cos 0
          0 0 1 ] */
	dx = 0;
	dy = 0;
	angle = radians;
       
    
}

rigid2d::Transform2D::Transform2D(const Vector2D & trans, double radians){ 
        /* [cos -sin dx
         sin cos dy
          0 0 1 ] */
        dx=trans.x ;
	dy=trans.y ;
	angle = radians;
      

        }

        

rigid2d::Vector2D rigid2d::Transform2D::operator()(rigid2d::Vector2D v) const{ /* const so need to create a new vector2d  */

                
                /* [cos -sin   dx 
                  sin   cos   dy]
                  0     0      1]   */
                Vector2D vnew;

                /*  due to rotation  */
                vnew.x = v.x * cos(this->angle) - v.y * sin(this->angle) ;
	        vnew.y = v.x * sin(this->angle )  + v.y * cos(this->angle) ; 

                /* due to translation  */
                vnew.x=vnew.x+this->dx;
                vnew.y=vnew.y+this->dy;
                return vnew;
        }



rigid2d::Transform2D rigid2d::Transform2D::inv() const{ /* const so need new transform to perform operation  */
                
        rigid2d::Transform2D t = *this; 
        
        t.angle=atan2(-sin(this->angle),cos(this->angle));   /*get tan inverse of negative angle */


        /*[ inv(R)  -inv(R)*p
             0        1   ]   */

        t.dx =-( this->dx * cos(t.angle) - this->dy * sin(t.angle ));
        t.dy = -(this->dx * sin(t.angle )  + this->dy * cos(t.angle)) ; 
        return t;
        }

       
rigid2d::Transform2D rigid2d::Transform2D::integrateTwist(const rigid2d::Twist2D & twist) {
     
        rigid2d::Twist2D s ;
        double theta=0;
    
        /* case 1  angular z is zero */
        if (twist.angle_z==0){
                s.angle_z=0;
                double magnitude=sqrt(pow(twist.linear_vx,2)+pow(twist.linear_vx,2));
                theta=sqrt(pow(twist.linear_vx,2)+pow(twist.linear_vx,2));
                if (magnitude==0){ /* when both linear vx and vy is zero */
                s.linear_vx=0;
                s.linear_vy=0;
                }
                else{
                s.linear_vx=twist.linear_vx/magnitude;  //computing for unit time 
                s.linear_vy=twist.linear_vy/magnitude; 
                }
   
        }

        else{
        /* case 2 angular z is not zero*/
        
        /* normalise angle */
        // double normalize_angle=rigid2d::normalize_angle(twist.angle_z);
        theta=abs(twist.angle_z);
        s.angle_z=twist.angle_z/theta;
        s.linear_vx=twist.linear_vx/theta;
        s.linear_vy=twist.linear_vy/theta;
        
        }

        /* [I  v*theta]
        // [0     1]
        // Iθ + [(1 − cos θ)[ω]+(θ − sin θ)[ω]^2] */
        Transform2D tbo;
        tbo.angle=atan2(sin(theta)*s.angle_z, 1 - (1 - cos(theta)) * pow(s.angle_z, 2));
        tbo.dx=s.linear_vx* (theta + (theta - sin(theta))*(-1)*pow(s.angle_z, 2))+ s.linear_vy* (1 - cos(theta))*(-1)*(s.angle_z);
        tbo.dy=s.linear_vy* (theta + (theta - sin(theta))*(-1)*pow(s.angle_z, 2))+ s.linear_vx* (1 - cos(theta)) * s.angle_z;

        // return transformation in original frame
        return *this*tbo;
}

     
std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Transform2D & tf){
        double deg=rigid2d::rad2deg(tf.angle);

        os << " "<< tf.dx ;
        os << " "<< tf.dy ;
        os << " "<< deg ;
        return os;
        }
  
std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Transform2D & tf){
        // std::cout << "input angle in degrees:" << std::endl;
        // is >> tf.angle;
        double deg;
        is >> deg;

        // std::cout << "new transformation of a rigid body following a twist for one time unitinpu/t dx :" << std::endl;
        is >> tf.dx;

        // std::cout << "input dy:" << std::endl;
        is >> tf.dy;

        tf.angle=rigid2d::deg2rad(deg);

        return is;
}


rigid2d::Transform2D & rigid2d::Transform2D::operator*=(const rigid2d::Transform2D & rhs){  /*rhs is constant so cannot modify that   so T=T*rhs*/
        this->angle=this->angle+rhs.angle;
        /* rotate the vector in that the new direcrion  and then translate */
        this->dx = cos(this->angle)*rhs.dx- sin(this->angle)*rhs.dy +this->dx;
        this->dy = sin(this->angle )*rhs.dx + cos(this->angle)*rhs.dy +this->dy;
        return *this;
        }




rigid2d::Transform2D rigid2d::operator*(rigid2d::Transform2D lhs, const rigid2d::Transform2D & rhs){/*  rhs is constant but lhs can be modified */ 
                
                lhs*=rhs;  /* lhs=lhs*rhs  */
                return lhs;
}


std::ostream & rigid2d::operator<<(std::ostream & os, const rigid2d::Twist2D & twist){
        double deg=rigid2d::rad2deg(twist.angle_z);
        os << " "<< deg;
        os << " "<< twist.linear_vx ;
        os << " "<< twist.linear_vy ;
        return os;

}

std::istream & rigid2d::operator>>(std::istream & is, rigid2d::Twist2D & twist){

        /* following convention of mordern robotics */
        // std::cout << " " << std::endl;
        double angle;
        is >> angle;
        twist.angle_z=rigid2d::deg2rad(angle);

        // std::cout << " " << std::endl;
        is >> twist.linear_vx;

        // std::cout << " " << std::endl;
        is >> twist.linear_vy;
        
        return is;
}




rigid2d::Twist2D rigid2d::Transform2D::gettwist_inframe(const rigid2d::Twist2D & twist, const rigid2d::Transform2D & tf){ /* both are constant so create new twist */

        /* [ 1 0 0 
            x cos -sin
           y sin cos]  */
        rigid2d::Twist2D t;
        t.angle_z=twist.angle_z ; // no changes in z direction since tf(x,y) is 2d
        t.linear_vx= tf.dx*t.angle_z+twist.linear_vx* cos(tf.angle) - twist.linear_vy * sin(tf.angle);
        t.linear_vy = tf.dy*t.angle_z +twist.linear_vy * sin(tf.angle )  + twist.linear_vy * cos(tf.angle) ; 
        return t;
}



