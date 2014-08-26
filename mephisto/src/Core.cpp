#include "Core.h"

Messenger::Messenger(){

    control_channel = nh_.resolveName("cmd_vel");
    navdata_channel = nh_.resolveName("ardrone/navdata");
    takeoff_channel = nh_.resolveName("ardrone/takeoff");
    land_channel = nh_.resolveName("ardrone/land");
    toggleState_channel = nh_.resolveName("ardrone/reset");

	std::string val;
	float valFloat;

	ros::param::get("~minPublishFreq", val);
	if(val.size()>0)
		sscanf(val.c_str(), "%f", &valFloat);
	else
		valFloat = Global::rate;
	minPublishFreq = valFloat;
	std::cout << "set minPublishFreq to " << valFloat << "ms"<< std::endl;

	// channels
	navdata_sub = nh_.subscribe(navdata_channel, 10, &Messenger::navdataCb, this);
	vel_pub	   = nh_.advertise<geometry_msgs::Twist>(control_channel,1);
	takeoff_pub	   = nh_.advertise<std_msgs::Empty>(takeoff_channel,1);
	land_pub	   = nh_.advertise<std_msgs::Empty>(land_channel,1);
	toggleState_pub	   = nh_.advertise<std_msgs::Empty>(toggleState_channel,1);
}

Messenger::~Messenger(){
}

// ROS message callbacks
void Messenger::navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr){
	this->navdata = *(navdataPtr.get());
	
	list<NavdataListener*>::iterator i;
	for(i = navdataListenerList.begin(); i != navdataListenerList.end(); ++i){
		(*i)->navdataCb(navdataPtr);
	}
}

void Messenger::addNavdataListener(NavdataListener* l){
	navdataListenerList.push_back(l);
}

// control drone functions
void Messenger::sendControlToDrone(ControlCommand cmd){
	geometry_msgs::Twist cmdT;
	cmdT.angular.z = -cmd.yaw;
	cmdT.linear.z = cmd.gaz;
	cmdT.linear.x = -cmd.pitch;
	cmdT.linear.y = -cmd.roll;
	
	cmdT.angular.x = cmd.hover ? 0 : 1;
	cmdT.angular.y = cmd.hover ? 0 : 1;

	vel_pub.publish(cmdT);
}

void Messenger::sendLand(){
	land_pub.publish(std_msgs::Empty());
}

void Messenger::sendTakeoff(){
	takeoff_pub.publish(std_msgs::Empty());	
}

void Messenger::sendToggleState(){
	toggleState_pub.publish(std_msgs::Empty());
}