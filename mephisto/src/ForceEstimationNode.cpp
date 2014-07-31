#include "ros/ros.h"
#include "Core.h"
#include "ForceEstimator.h"
#include <iostream>
#include <csignal>

bool quit = false;

int main(int argc, char* argv[]){
	ros::init(argc, argv, "drone_force_estimator");
	ros::Time::init();
	ros::Rate r(Global::rate);
	
	ForceEstimator* estimator = new ForceEstimator();
	
	signal(SIGINT, [](int signum) {
		std::cout << "\nEstimator shutdown" << std::endl;
		quit = true;
	});
	
	while(!quit){
		ros::spinOnce();
		r.sleep();
	}

	delete estimator;

	return 0;
}
