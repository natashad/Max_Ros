#include "ros/ros.h"
#include "Core.h"
#include "HeightController.h"
#include <iostream>
#include <csignal>

bool quit = false;

int main(int argc, char* argv[]){
	ros::init(argc, argv, "drone_height_controller");
	ros::Time::init();
	ros::Rate r(Global::rate);
	
	Messenger* messenger = new Messenger();
	SimpleController* controller = new SimpleController(messenger);
	
	signal(SIGINT, [](int signum){
		std::cout << "\nHeight controller shutdown" << std::endl;
		quit = true;
	});
	
	controller->trackHeight(1000.0);
	
	while(!quit){
		ros::spinOnce();
		r.sleep();
	}
	
	delete messenger;
	delete controller;

	return 0;
}
