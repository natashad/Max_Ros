#ifndef __FORCEESTIMATOR_H
#define __FORCEESTIMATOR_H

#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "Core.h"
#include <ardrone_autonomy/Navdata.h>
#include <string>
#include <iostream>
#include <complex>

using namespace ros;
using namespace geometry_msgs;
using namespace ardrone_autonomy;


//Length of the arrays for filtering purpose
#define N 10

class ForceEstimator {
	private:
	Twist force;
	
	//needed for signal processing
	Twist inputs[N];
	Twist outputs[N];
	Twist outputs2[N];

	//needed to calculate angular accel
	Vector3 rpy;
	Vector3 omega;
	//their values at n-1
	Vector3 rpy_;
	Vector3 omega_;

	NodeHandle nh_;
	int minPublishFreq;
	
	Publisher force_pub;
	string force_pub_channel;
	
	Subscriber navdata_sub;
	string navdata_sub_channel;
	
	public:
	
	ForceEstimator();
	~ForceEstimator();
	
	Twist getForce(){return force;};
	Vector3 getTransForce(){ return force.linear;};
	Vector3 getRotForce(){ return force.angular;};
	
	void estimate(const NavdataConstPtr navdata);
	void publish();
};

#endif