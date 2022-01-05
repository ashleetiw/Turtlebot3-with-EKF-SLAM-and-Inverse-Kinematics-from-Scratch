#define CATCH_CONFIG_RUNNER
#include <catch_ros/catch.hpp>
#include "rigid2d/rigid2d.hpp"
#include <iostream>
#include <cmath>
#include "rigid2d/diff_drive.hpp"


TEST_CASE("almost equal function"){
	/*  test case for almost_equal function */
	REQUIRE(rigid2d::almost_equal(5, 5)==true);

	REQUIRE(rigid2d::almost_equal(1, -1)==false);
}


TEST_CASE(" degree to rad  function"){
	/*  test case for converting degree to rad */
	REQUIRE(round(100*rigid2d::deg2rad(180))==314);   // round(100*3.14)
	REQUIRE(round(100*rigid2d::deg2rad(90))==157);
	
}


TEST_CASE(" rad to degree function"){
	/*  test case for  converting rad to degree */
	REQUIRE(round(100*rigid2d::deg2rad(180))==314);   // round(100*3.14)
	REQUIRE(round(100*rigid2d::deg2rad(90))==157);
	
}

/////////////////////////////////////////////   Vector 2D//////////////////////////////////////////

TEST_CASE("vector"){

	/*  test case for declaring Vectro2D */
	// test 1
	rigid2d::Vector2D v;
	v.x=3;
	v.y=2;
    REQUIRE(v.x==3);
	 REQUIRE(v.y==2);

	// test 2
	rigid2d::Vector2D v1;
	v1.x=-10;
	v1.y=0;
    REQUIRE(v1.x==-10 );
	REQUIRE(v1.y==0);
}



TEST_CASE("vector normalize"){
	/*  test case for normalizing a vector2D */
	// test 1
	rigid2d::Vector2D v;
	v.x=4;
	v.y=3;

	rigid2d::Vector2D v_norm;
	rigid2d::Vector2D vn= v_norm.normalize(v);
	rigid2d::Vector2D v1;
	v1.x=0.8;
	v1.y=0.6;
    REQUIRE(vn.x==v1.x);
	REQUIRE(vn.y==v1.y);


	// test 2
	rigid2d::Vector2D vt;
	vt.x=8;
	vt.y=6;
	rigid2d::Vector2D vtn= v_norm.normalize(vt);
	REQUIRE(vtn.x==v1.x);
	REQUIRE(vtn.y==v1.y);


}


TEST_CASE("Vector2D operator <<"){
	/*  test case for output vector2D */
	// test 1
	std::stringstream stream;
	rigid2d::Vector2D v;
	v.x=3;
	v.y=2;
	stream << v;
	REQUIRE(stream.str()=="32");

	// test2
	std::stringstream streamt;
	rigid2d::Vector2D v1;
	v1.x=10;
	v1.y=-2;
	streamt << v1;
	REQUIRE(streamt.str()=="10-2");

}


TEST_CASE("Vector2D operator >>"){
	/*  test case for input vector2D */
	// test 1
	std::stringstream stream;
	stream.str("2 6");
	rigid2d::Vector2D v;
	stream >> v;
	REQUIRE(v.x==2);
	REQUIRE(v.y==6);

	// test 2
	std::stringstream streamt;
	streamt.str("10 -10");
	rigid2d::Vector2D v1;
	streamt >> v1;
	REQUIRE(v1.x==10);
	REQUIRE(v1.y==-10);

}

///////////////////////////// / /////////  Transformation 2D ///////////////////////////////



TEST_CASE("Transform2D translation "){
		/*  test case for Transform2D pure translation */
		// test 1

		rigid2d::Vector2D v;
		v.x=4;
		v.y=1;
		rigid2d::Transform2D t(v);
		REQUIRE(t.dx==4);
		REQUIRE(t.dy==1);
		REQUIRE(t.angle==0);

		//test 2
		rigid2d::Vector2D v1;
		v1.x=-2;
		v1.y=3;
		rigid2d::Transform2D t1(v1);
		REQUIRE(t1.dx==-2);
		REQUIRE(t1.dy==3);
		REQUIRE(t1.angle==0);

}


TEST_CASE("Transform2D rotation "){
		/*  test case for Transform2D pure rotation */
		// test 1
		double angle=3.14;
		rigid2d::Transform2D t(angle);
		REQUIRE(t.dx==0);
		REQUIRE(t.dy==0);
		REQUIRE(t.angle==3.14);

		// test 2

		double angle1=0;
		rigid2d::Transform2D t1(angle1);
		REQUIRE(t1.dx==0);
		REQUIRE(t1.dy==0);
		REQUIRE(t1.angle==0);

}


TEST_CASE("Transform2D  translation + rotation "){
		/*  test case for Transform2D translation + rotation */
		// test 1
		rigid2d::Vector2D v;
		v.x=1;
		v.y=4;
		double angle=0;
		rigid2d::Transform2D t(v,angle);
		REQUIRE(t.dx==1);
		REQUIRE(t.dy==4);
		REQUIRE(t.angle==0);

		// test 2
		rigid2d::Vector2D vt;
		vt.x=-2;
		vt.y=0;
		double anglet=1.57;
		rigid2d::Transform2D t1(vt,anglet);
		REQUIRE(t1.dx==-2);
		REQUIRE(t1.dy==0);
		REQUIRE(round(100*t1.angle)==157);   //round(1.57*100)

}



TEST_CASE("Transform2D  "){
	/*  test case for Transform2D declaration */
	rigid2d::Transform2D t1;
		REQUIRE(t1.dx==0);
		REQUIRE(t1.dy==0);
		REQUIRE(t1.angle==0);

}


TEST_CASE("apply transformation to Vector2D "){
	/*  test case for applying Transform2D on a vector2D */
	//test 1
	rigid2d::Transform2D t;
	t.dx=3;
	t.dy=2;
	t.angle=0;
	rigid2d::Vector2D v;
	v.x=4;
	v.y=1;

	rigid2d::Vector2D v2=t(v);

	rigid2d::Vector2D v1;  // hand calculation
	v1.x=7;
	v1.y=3;

	REQUIRE(v2.x==v1.x);
	REQUIRE(v2.y==v1.y);


	//test 2
	rigid2d::Transform2D tn;
	tn.dx=1;
	tn.dy=4;
	tn.angle=1.57;
	rigid2d::Vector2D vn;
	vn.x=2;
	vn.y=5;

	rigid2d::Vector2D v2n=tn(vn);

	rigid2d::Vector2D v1n;  // hand calculation
	v1n.x=-4;
	v1n.y=6;

	REQUIRE(round(v2n.x)==v1n.x);
	REQUIRE(round(v2n.y)==v1n.y);

	
}

TEST_CASE("Transform2D inverse"){
	/*  test case for inversing Transform2D */
	//test 1
	rigid2d::Transform2D t;
	t.dx=-1;
	t.dy=3;
	t.angle=0;
	
	rigid2d::Transform2D t1;   // identity matrix
	t1.dx=0;
	t1.dy=0;
	t1.angle=0;

	
	rigid2d::Transform2D tin=t.inv();
	rigid2d::Transform2D t2=t*tin;   // t* tinv=Identity matrix
	REQUIRE(t1.dx==t2.dx);
	REQUIRE(t1.dy==t2.dy);
	REQUIRE(t1.angle==-t2.angle);

	// //test 2
	rigid2d::Transform2D ta;
	ta.dx=3;
	ta.dy=1;
	ta.angle=1.57;

	rigid2d::Transform2D tinva=ta.inv();
	rigid2d::Transform2D t3;
	t3.dx=-1;
	t3.dy=3;
	t3.angle=-1.57;


	REQUIRE(round(tinva.dx)==t3.dx);
	REQUIRE(round(tinva.dy)==t3.dy);
	REQUIRE(tinva.angle==t3.angle);
	// additional check
	// REQUIRE(round(tinva.angle)==-round(ta.angle));  // angle will be -ve 

}



TEST_CASE("Transform2D  multiplication"){
	/*  test case for multiplication of 2 Transform2D  */
	//  test 1
	rigid2d::Transform2D t;
	t.dx=3;
	t.dy=1;
	t.angle=0;

	rigid2d::Transform2D t1;
	t1.dx=1;
	t1.dy=2;
	t1.angle=1.57;

	rigid2d::Transform2D t2;    
	t2.dx=1;
	t2.dy=2;
	t2.angle=1.57;

	rigid2d::Transform2D t3=t*t1;

	REQUIRE(int(t3.dx)==int(t2.dx));
	REQUIRE(int(t3.dy)==int(t2.dy));
	REQUIRE(int(t3.angle)==int(t2.angle));


	//test 2
	rigid2d::Transform2D ta;
	ta.dx=5;
	ta.dy=6;
	ta.angle=0;

	rigid2d::Transform2D tb;
	tb.dx=-8;
	tb.dy=2;
	tb.angle=0;

	rigid2d::Transform2D tc;
	tc.dx=-3;
	tc.dy=8;
	tc.angle=0;

	rigid2d::Transform2D tx=ta*tb;
	REQUIRE(int(tx.dx)==int(tc.dx));
	REQUIRE(int(tx.dy)==int(tc.dy));
	REQUIRE(int(tx.angle)==int(tc.angle));


}


TEST_CASE("Transform2D operator <<"){
	/*  test case for Transform2D output */
	//test1
	std::stringstream stream;
	rigid2d::Transform2D t;
	t.dx=5;
	t.dy=6;
	t.angle=0;
	// double deg=rigid2d::rad2deg(3.14)
	stream << t;
	REQUIRE(stream.str()==" 5 6 0");


	//test2
	std::stringstream streamt;
	rigid2d::Transform2D t1;
	t1.dx=-1;
	t1.dy=8;
	t1.angle=1.57;
	// double deg=rigid2d::rad2deg(3.14)
	streamt << t1;
	REQUIRE(streamt.str()==" -1 8 89.9544");

}


TEST_CASE("Transform2D operator >>"){
	/*  test case for Transform2D input  */
	// test 1
	std::stringstream stream;
	rigid2d::Transform2D t;
	stream.str("0 8 9");
	// double deg=rigid2d::rad2deg(3.14)
	stream >> t;
	REQUIRE(t.angle==0);
	REQUIRE(t.dx==8);
	REQUIRE(t.dy==9);


	// test 2
	std::stringstream streamt;
	rigid2d::Transform2D t1;
	streamt.str("180 -2 7");
	// double deg=rigid2d::rad2deg(3.14)
	streamt >> t1;
	REQUIRE(round(t1.angle)== 3);
	REQUIRE(t1.dx==-2);
	REQUIRE(t1.dy==7);
	

}

//////////////////////  Twist //////////////////////////


TEST_CASE("Twist2D operator <<"){
	/*  test case for Twist2D output */
	// test 1
	std::stringstream stream;
	rigid2d::Twist2D t;
	t.linear_vx=5;
	t.linear_vy=6;
	t.angle_z=0;
	// double deg=rigid2d::rad2deg(3.14)
	stream << t;
	REQUIRE(stream.str()==" 0 5 6");

	// test2
	std::stringstream streamt;
	rigid2d::Twist2D t1;
	t1.linear_vx=8;
	t1.linear_vy=9;
	t1.angle_z=1.57;
	// double deg=rigid2d::rad2deg(3.14)
	streamt << t1;
	REQUIRE(streamt.str()==" 89.9544 8 9");

}


TEST_CASE("Twist2D operator >>"){
	/*  test case for Twist2D input  */
	// test 1
	std::stringstream stream;
	rigid2d::Twist2D t;
	
	stream.str("90 4 7");
	stream >> t;
	REQUIRE(t.linear_vx==4);
	REQUIRE(t.linear_vy==7);
	REQUIRE(round(t.angle_z*100)==157);


	// test 2
	std::stringstream streamt;
	rigid2d::Twist2D t1;
	
	streamt.str("0 -1 1");
	streamt >> t1;
	REQUIRE(t1.linear_vx==-1);
	REQUIRE(t1.linear_vy==1);
	REQUIRE(round(t1.angle_z==0));

}


TEST_CASE("Twist 2D gettwistin frame"){
	/*  test case for to get Twist2D in frame   */
	// test 1
	rigid2d::Twist2D twist;
	twist.linear_vx=4;
	twist.linear_vy=3;
	twist.angle_z=0;

	rigid2d::Transform2D t;
	t.dx=5;
	t.dy=6;
	t.angle=0;

	rigid2d::Twist2D tnew= t.gettwist_inframe(twist,t);

	rigid2d::Twist2D twist1;
	twist1.linear_vx=4;
	twist1.linear_vy=3;
	twist1.angle_z=0;
    REQUIRE(tnew.linear_vx==twist1.linear_vx);
	REQUIRE(tnew.linear_vy==twist1.linear_vy);
	REQUIRE(tnew.angle_z==twist1.angle_z);

	// test 2

	rigid2d::Twist2D twistn;
	twistn.linear_vx=2;
	twistn.linear_vy=5;
	twistn.angle_z=0;

	rigid2d::Transform2D tn;
	tn.dx=-1;
	tn.dy=0;
	tn.angle=0;

	rigid2d::Twist2D tnew1= tn.gettwist_inframe(twistn,tn);

	rigid2d::Twist2D twist1n;
	twist1n.linear_vx=2;
	twist1n.linear_vy=5;
	twist1n.angle_z=0;

	REQUIRE(tnew1.linear_vx==twist1n.linear_vx);
	REQUIRE(tnew1.linear_vy==twist1n.linear_vy);
	REQUIRE(tnew1.angle_z==twist1n.angle_z);



}


/*****************  hw 2 tests **************************************/

TEST_CASE(" addition operator +="){
	rigid2d::Vector2D v1;
	v1.x=2;
	v1.y=0;

	rigid2d::Vector2D v2;
	v2.x=-1;
	v2.y=1;

	v1.operator+=(v2);

	REQUIRE(v1.x==1);
	REQUIRE(v1.y==1);
	
}


TEST_CASE(" subtraction operator  -="){
	rigid2d::Vector2D v1;
	v1.x=2;
	v1.y=0;

	rigid2d::Vector2D v2;
	v2.x=-1;
	v2.y=1;

	v1.operator-=(v2);

	REQUIRE(v1.x==3);
	REQUIRE(v1.y==-1);
	
}

TEST_CASE(" scalar multiplication operator  *="){
	rigid2d::Vector2D v1;
	v1.x=2;
	v1.y=0;

	double s=4;

	v1.operator*=(s);

	REQUIRE(v1.x==8);
	REQUIRE(v1.y==0);
	
}


TEST_CASE(" addition operator +"){
	rigid2d::Vector2D v1;
	v1.x=2;
	v1.y=0;

	rigid2d::Vector2D v2;
	v2.x=-1;
	v2.y=1;
	 

	rigid2d::operator+(v2,v1);

	REQUIRE(v1.x==1);
	REQUIRE(v1.y==1);
	
}


TEST_CASE(" subtraction operator -"){
	rigid2d::Vector2D v1;
	v1.x=2;
	v1.y=0;

	rigid2d::Vector2D v2;
	v2.x=-1;
	v2.y=1;
	 

	rigid2d::operator-(v2,v1);

	REQUIRE(v1.x==3);
	REQUIRE(v1.y==-1);
	
}


TEST_CASE(" multiplication operator *"){
	rigid2d::Vector2D v1;
	v1.x=2;
	v1.y=0;

	double s=4;
	 

	rigid2d::operator*(v1,s);

	REQUIRE(v1.x==8);
	REQUIRE(v1.y==0);
	
}

TEST_CASE(" calculate magnitude and angle "){
	rigid2d::Vector2D v1;
	v1.x=2;
	v1.y=0;

	double m=v1.magnitude();
	double a=v1.angle();

	REQUIRE(m==2);
	REQUIRE(a==0);
}


TEST_CASE("normalize angle "){
  double PI=3.14159265358979323846;
  /* test 1*/

  double new_r=rigid2d::normalize_angle(4*PI);

  REQUIRE(new_r==0);

  /* test2 */

  double new_a=rigid2d::normalize_angle(3*PI/2);

  REQUIRE(new_a==-PI/2);
  
}


TEST_CASE("integrate twist frame"){
	/*compute the transformation corresponding to a rigid body 
	following a constant twist (in its original body frame) for one time unit */

	/* pure translation*/
	rigid2d::Twist2D twistt;
	twistt.linear_vx=1;
	twistt.linear_vy=100;
	twistt.angle_z=0;

	rigid2d::Transform2D t_translate;

	t_translate=t_translate.integrateTwist(twistt);

	REQUIRE(t_translate.dx==1);
	REQUIRE(round(t_translate.dy)==100);
	REQUIRE(t_translate.angle==0);
		


	// /* pure rotation */
	rigid2d::Twist2D twistr;
	twistr.linear_vx=0;
	twistr.linear_vy=0;
	twistr.angle_z=1.57;

	rigid2d::Transform2D t_rot;

	t_rot= t_rot.integrateTwist(twistr);

	REQUIRE(t_rot.dx==0);
	REQUIRE(t_rot.dy==0);
	REQUIRE(t_rot.angle==1.57);

	/* rotation + translation  example from mordern robotics */

	rigid2d::Twist2D twist;
	twist.linear_vx=3.14;
	twist.linear_vy=0;
	twist.angle_z=1.57;

	rigid2d::Transform2D t;

	t= t.integrateTwist(twist);

	REQUIRE(round(t.dx)==-2);
	REQUIRE(round(t.dy)==2);
	REQUIRE(t.angle==1.57);

}






// TEST_CASE("Diff()constructir"){
// 	rigid2d::DiffDrive d;
	

// }

TEST_CASE("TwistToWheels"){

  
	// Rotation Test
	rigid2d::Twist2D Vb;
	Vb.linear_vx=0;
	Vb.linear_vy=0;
	Vb.angle_z=1;
	rigid2d::DiffDrive d ;

	
	rigid2d::wheel_vel w= d.twisttowheel(Vb,0.2,1);
	REQUIRE(w.wl== -2.5);
  	REQUIRE(w.wr== 2.5);

  	// // Translation Test

	rigid2d::Twist2D Vt;
	Vt.linear_vx=1;
	Vt.linear_vy=0;
	Vt.angle_z=0;
	w= d.twisttowheel(Vt,0.2,1);
	REQUIRE(w.wl== 5.0);
  	REQUIRE(w.wr== 5.0);
  

  	// // Mixed Motion Test
	rigid2d::Twist2D Vm;
	Vm.linear_vx=1;
	Vm.linear_vy=0;
	Vm.angle_z=1;
	w= d.twisttowheel(Vm,0.2,1);
	REQUIRE(w.wl== 2.5);
  	REQUIRE(w.wr== 7.5);
  	
}


TEST_CASE("WheelsToTwist")
{   // Translation Test
	rigid2d::wheel_vel w;
	w.wl=10;
	w.wr=10;

	rigid2d::DiffDrive d;
	rigid2d::Twist2D t = d.wheeltotwist(w,0.2,1);
	REQUIRE(t.linear_vx== 2);
  	REQUIRE(t.linear_vy==0);
	REQUIRE(t.angle_z==0);
	
  	// Rotation Test
	w.wl = -10;
	w.wr = 10;
	t = d.wheeltotwist(w,0.2,1);
	REQUIRE(t.linear_vx== 0);
  	REQUIRE(t.linear_vy==0);
	REQUIRE(t.angle_z==4);
	
	// Mixed Motion Test
	w.wl = 0;
	w.wr = 10;
	t = d.wheeltotwist(w,0.2,1);
	REQUIRE(t.linear_vx== 1);
  	REQUIRE(t.linear_vy==0);
	REQUIRE(t.angle_z==2);

}


TEST_CASE("wheeltopose")
{	double PI=3.14159265358979323846;
	// Translation Test
	rigid2d::wheel_vel w;
	rigid2d::DiffDrive d;
	// both wheels rotate 2pi
	double left_wheel = 0.5 * PI;
	double right_wheel = 0.5 * PI;
	w = d.wheeltopose(left_wheel, right_wheel,0.2,1);
	REQUIRE(w.wl== PI / 2);
	REQUIRE(w.wr== PI / 2);
	REQUIRE(d.theta== 0.0);
	REQUIRE(round(d.x*100)==31);
	REQUIRE(d.y==0);

	// // Rotation Test

	left_wheel = -PI/4;
	right_wheel = PI/4;
	w = d.wheeltopose(left_wheel, right_wheel,0.2,1);
	REQUIRE(round(w.wl*100)==-79);
	REQUIRE(round(w.wr*100)==-236);
	REQUIRE(round(d.theta*100)==-31);
	REQUIRE(round(d.x)==0);
	REQUIRE(round(d.y)==0);
	

	// // // Mixed Motion Test

	left_wheel = 0;
	right_wheel = PI/4;
	w = d.wheeltopose(left_wheel, right_wheel,0.2,1);
	REQUIRE(round(w.wl)==0);
	REQUIRE(round(w.wr*100)==79);
	REQUIRE(round(10*d.theta)==-2);
	REQUIRE(round(d.x*100)==11);
	REQUIRE(round(d.y*100)==14);

}


TEST_CASE("update config based on twist")
{
	rigid2d::DiffDrive d;
	rigid2d::Twist2D Vb;
	Vb.linear_vx=1;
	Vb.linear_vy=0;
	Vb.angle_z=0;

// 	// Translation Test
	rigid2d::wheel_vel w;
	w=d.updateconfig(Vb,0.2,1);

	// pose= d.getpose();
	REQUIRE(d.theta== 0);
	REQUIRE(d.x==1);
	REQUIRE(d.y==0);

	// Rotation Test
	Vb.linear_vx=1;
	Vb.linear_vy=0;
	Vb.angle_z=0;
	w=d.updateconfig(Vb,0.2,1);
	REQUIRE(d.theta== 0);
	REQUIRE(d.x==3);
	REQUIRE(d.y==0);

// // 	// Mixed Motion Test
	Vb.linear_vx=1;
	Vb.linear_vy=0;
	Vb.angle_z=3.14/4;
	w=d.updateconfig(Vb,0.2,1);
	REQUIRE(round(100*d.theta)==79);
	REQUIRE(round(d.x)==6);
	REQUIRE(round(100*d.y)==90);

}

int main(){
	
	Catch::Session session;
    // ros::init(argc, argv, "test_node_rigid2d");
    // ros::NodeHandle nh;
    return session.run();
}



