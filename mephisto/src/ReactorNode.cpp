#include "ros/ros.h"
#include "Core.h"
#include "State.h"
#include <iostream>
#include <cstdlib>
#include <csignal>

bool quit = false;

int main(int argc, char* argv[]){
	int rosargc = 1;
	ros::init(rosargc, argv, "drone_reactor");
	ros::Time::init();
	ros::Rate r(Global::rate);
	
	Messenger* messenger = new Messenger();
	StateMachine* fsm;
	if(argc > 1){
		if(atoi(argv[1]) == 1){
			fsm = new StateMachine(messenger, true);
			std::cout << "CAUTION: allowing takeoff" << std::endl;
		}else{
			fsm = new StateMachine(messenger);
			std::cout << "not allowing takeoff" << std::endl;
		}
	}else{
		fsm = new StateMachine(messenger);
		std::cout << "not allowing takeoff" << std::endl;
	}
	
	signal(SIGINT, [](int signum) {
		std::cout << "\nReactor shutdown" << std::endl;
		quit = true;
	});
	
	
	// In case we need to reset after emergency landing or crash
	messenger->sendToggleState();
	
	// Calibrate the IMU and flat trim
//	ros::service::call("ardrone/flattrim");
//	ros::service::call("ardrone/imu_recalib");
	
	while(!quit){
		ros::spinOnce();
		fsm->execute();
		r.sleep();
	}

	// Send a hover command
	messenger->sendControlToDrone(ControlCommand());
	
	// Land
	messenger->sendLand();

	delete messenger;
	delete fsm;

	return 0;
}
