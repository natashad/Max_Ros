#include "HeightController.h"


//Constructor to connect to the messenger
SimpleController::SimpleController(Messenger* messenger) :
z(0), e_sum(0)
{
	this->messenger = messenger;
	messenger->addNavdataListener(this);

}

//Destructor, removing reference to messenger
SimpleController::~SimpleController(){
	messenger = 0;
}

//Set the height to be tracked
void SimpleController::trackHeight(double z){
	this->z = z;
}

/**
* Every time we get a navdata update from the drone
*/
void SimpleController::navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr){
	
	double e = navdataPtr->altd - z;
	e_sum += e;
	
	double gaz = (-K_P * e)/2000;
	
	std::cout << gaz << std::endl;
	
	messenger->sendControlToDrone(ControlCommand(0, 0, 0, gaz));
}
