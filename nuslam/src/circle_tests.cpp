#define CATCH_CONFIG_RUNNER
#include <ros/ros.h>
#include <catch_ros/catch.hpp>
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/LaserScan.h>
#include "rigid2d/rigid2d.hpp"
#include <cmath>
#include <armadillo>
// #include "landmarks.cpp"


TEST_CASE(" test 1circle fitting")
{

	REQUIRE(rigid2d::almost_equal(2,0));
// 	double test_threshold = 1e-4;

// 	double cluster_threshold = 1e6; // large threshold to fit any circle

// 	// TEST 1 INPUTS: {(1, 7), (2, 6), (5, 8), (7, 7), (9, 5), (3, 7)}
// 	//		  OUTPUTS: center (4.615482, 2.807354) radius 4.8725
// 	std::vector<rigid2d::Vector2D> arr;
// 	rigid2d::Vector2D p1, p2,p3,p4,p5,p6;

// 	p1.x=1;
// 	p1.y=7;
// 	arr.push_back(p1);

// 	p2.x=2;
// 	p2.y=6;
// 	arr.push_back(p2);	

// 	p3.x=5;
// 	p3.y=8;
// 	arr.push_back(p3);

// 	p4.x=7;
// 	p4.y=7;
// 	arr.push_back(p4);

// 	p5.x=9;
// 	p5.y=5;
// 	arr.push_back(p5);

// 	p6.x=3;
// 	p6.y=7;
// 	arr.push_back(p6);


// 	std::vector<rigid2d::Vector2D> coords;
// 	std::vector<double> radius;
// 	fit_circle(arr,coords,radius);
	
// // 	// Now evaluate centre and radius
	
// 	REQUIRE(coords.at(0).x==4.615482);
// 	REQUIRE(coords.at(0).y== 2.807354);
// 	REQUIRE(radius.at(0)== 4.8275);

}


// TEST_CASE(" test 2circle fitting")
// {

// 	double test_threshold = 1e-4;

// 	double cluster_threshold = 1e6; // large threshold to fit any circle

	
// 	std::vector<rigid2d::Vector2D> arr;
// 	rigid2d::Vector2D p1, p2,p3,p4,p5,p6;

// 	p1.x=-1;
// 	p1.y=0;
// 	arr.push_back(p1);

// 	p2.x=-0.3;
// 	p2.y=-0.06;
// 	arr.push_back(p2);	

// 	p3.x=0.3;
// 	p3.y=0.1;
// 	arr.push_back(p3);

// 	p4.x=1;
// 	p4.y=0;
// 	arr.push_back(p4);


// 	std::vector<rigid2d::Vector2D> coords;
// 	std::vector<double> radius;
// 	fit_circle(arr,coords,radius);
	
// 	// Now evaluate centre and radius
	
// 	REQUIRE(coords.at(0).x==0.4908357);
// 	REQUIRE(coords.at(0).y== -22.15212);
// 	REQUIRE(radius.at(0)== 22.17979);

// }