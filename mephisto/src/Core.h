#ifndef __MEPHISTO_CORE_H
#define __MEPHISTO_CORE_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include <ardrone_autonomy/Navdata.h>
#include <string>
#include <list>
#include <cmath>

namespace Global{
	//Rate in hertz of how fast everything should happen
	const int rate = 200;
};

using namespace ros;
using namespace std;

struct ControlCommand
{
	inline ControlCommand() { roll = pitch = yaw = gaz = 0; hover = true; }
	inline ControlCommand(double roll, double pitch, double yaw, double gaz)
	{
		this->roll = roll;
		this->pitch = pitch;
		this->yaw = yaw;
		this->gaz = gaz;
		hover = true;
	}
	inline ControlCommand(double roll, double pitch, double yaw, double gaz, bool hover)
	{
		this->roll = roll;
		this->pitch = pitch;
		this->yaw = yaw;
		this->gaz = gaz;
		this->hover = hover;
	}
	double yaw, roll, pitch, gaz;
	bool hover;
};

class NavdataListener {
	public:
	virtual void navdataCb(const ardrone_autonomy::NavdataConstPtr) = 0;
};

class Messenger {
private:
	Subscriber navdata_sub;
	Publisher vel_pub;
	Publisher takeoff_pub;
	Publisher land_pub;
	Publisher toggleState_pub;


	// parameters
	string control_channel;
	string navdata_channel;
	string command_channel;
	string land_channel;
	string takeoff_channel;
	string toggleState_channel;

	ardrone_autonomy::Navdata navdata;
	
	list<NavdataListener*> navdataListenerList;

public:
	Messenger();
	~Messenger();
	
	NodeHandle nh_;
	int minPublishFreq;

	// ROS message callbacks
	void navdataCb(const ardrone_autonomy::NavdataConstPtr);
	void addNavdataListener(NavdataListener*);

	// control drone functions
	void sendControlToDrone(ControlCommand);
	void sendLand();
	void sendTakeoff();
	void sendToggleState();

};

#endif