#ifndef __SIMPLECONTROLLER_H
#define __SIMPLECONTROLLER_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "Core.h"
#include <ardrone_autonomy/Navdata.h>
#include <string>
#include <iostream>

#define K_P		10.0
#define	K_I 	100.0

using namespace ros;

class SimpleController : public NavdataListener {
	private:
	
	//Height to track
	double z;
	
	//Running sum of error terms
	double e_sum;
	
	//Pointer to the messenger
	Messenger* messenger;

	
	public:
	
	SimpleController(Messenger* messenger);
	~SimpleController();
	
	void trackHeight(double z);
	
	void navdataCb(const ardrone_autonomy::NavdataConstPtr);
};

#endif
