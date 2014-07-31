#include "State.h"
#include <cmath>

//==============================================================================
// Hover State
//==============================================================================

/*
	Hover state is the neutral state where the quad is just hovering and 
	accepting input
*/

void HoverState::enter(Navdata nav, Twist f){
	std::cout << "Entering HOVER state" << std::endl;
	blink_green();
	cmdsent = false;
}

void HoverState::exit(Navdata n, Twist f){}

ControlCommand* HoverState::perform(Navdata navdata, Twist force){
	//Hover command
	if(!cmdsent){
	    cmdsent = true;
	    return new ControlCommand(0, 0, 0, 0);
	}else
	    return 0;
}

bool HoverState::isLocked(){ 
	return false;
}


//==============================================================================
// Passive State
//==============================================================================
/*
	Once the input is accepted and a new state is calculated, the quad goes into
	this state to bring its orientation back to normal before proceeding to 
	execute stuff. May not be needed.
	
	Guess not, instead use this state for awaiting double tap commands
*/
void PassiveState::enter(Navdata n, Twist f){
	std::cout << "Entering PASSIVE state" << std::endl;
	start = ros::Time::now();
	stage = 0;
	prevForce = f;
	blink_orange();
}

void PassiveState::exit(Navdata n, Twist f){
	triggerType = INDETERMINATE;
	stage = 0;
}

ControlCommand* PassiveState::perform(Navdata navdata, Twist force){
	//First we need to let the inputs fall below 70% of the threshold
	if(stage == 0){
		if(magnitude(force.linear) < 0.7*LIN_THRESHOLD && magnitude(force.angular) < 0.7*ROT_THRESHOLD){
			stage = 1;
		}
	}
	//Then we must look for another peak
	else if(stage == 1){
		if(magnitude(force.linear) > LIN_THRESHOLD || magnitude(force.angular) > ROT_THRESHOLD){
			//Look for a peak in the force data
			if(magnitude(force.linear) < magnitude(prevForce.linear) || 
				magnitude(force.angular) < magnitude(prevForce.angular)){
				//peak found at prev force;
			
				ForceType type = getForceType(prevForce);
				//detected another event of the same type			
				if(type	== triggerType){
					stage = 2;
					steadyFlightStart = ros::Time::now();
				}
			}
		}
	}
	//If found, let the inputs fall below stable threshold before transitioning
	else if(stage == 2){
		if(magnitude(force.linear) > LIN_STABLE || magnitude(force.angular) > ROT_STABLE){
			steadyFlightStart = ros::Time::now();
		}else if((ros::Time::now() - steadyFlightStart).toSec() >= DURATION_STABLE ){
			stage = 3;
		}
	}
	
	prevForce = force;
	
	//Hover controller on
	return new ControlCommand(0, 0, 0, 0);
}

bool PassiveState::isLocked(){
	if(stage == 2){
		return true;
	}else if(stage == 3){
		return false;
	}else{
		return ((ros::Time::now() - start).toSec() < DOUBLE_TAP_TIME);
	}
}

//==============================================================================
// Animation State
//==============================================================================

/*
	State for executing a flight animation. 
*/
AnimationState::AnimationState(){
}
void AnimationState::enter(Navdata n, Twist f){
	std::cout << "Entering ANIMATION state" << std::endl;
	initialForce = f;
	executed = false;
	finished = false;
}

void AnimationState::exit(Navdata n, Twist f){
}

ControlCommand* AnimationState::perform(Navdata navdata, Twist force){
	//If the animation has been executed we want to wait for the quad to 
	//stabilize itself before returning to the hover state
	if(executed){
		if(magnitude(force.linear) > LIN_STABLE || magnitude(force.angular) > ROT_STABLE){
			steadyFlightStart = ros::Time::now();
		}else if((ros::Time::now() - steadyFlightStart).toSec() >= DURATION_STABLE){
			finished = true;
		}
	}else{
		FlightAnim flight;
		LedAnim light;
	
		flight.request.type = UNDEFINED_TYPE;
		light.request.type = UNDEFINED_TYPE;
	
		ForceType ftype = getForceType(initialForce);
		
		switch(ftype){
			case TWIST_CCW:
				flight.request.type = YAW_SHAKE;	//yaw shake
				light.request.type = FIRE;
				break;
			case TWIST_CW:
				flight.request.type = YAW_SHAKE;	//yaw shake
				light.request.type = FIRE;
				break;
			case ECCENTRIC_PUSH:
				flight.request.type = YAW_DANCE;	//yaw dance
				light.request.type = FIRE;
				break;
			case FRONT_PUSH:
				flight.request.type = THETA_DANCE;
				light.request.type = FIRE;
				break;
			case BACK_PUSH:
				flight.request.type = THETA_DANCE;
				light.request.type = FIRE;
				break;
			case LEFT_PUSH:
				flight.request.type = PHI_DANCE;
				light.request.type = FIRE;
				break;
			case RIGHT_PUSH:
				flight.request.type = PHI_DANCE;
				light.request.type = FIRE;
				break;
			case TOP_PUSH:
				//We never should be in here, because it would be a land command
				flight.request.type = VZ_DANCE;
				light.request.type = FIRE;
				break;
			case BOTTOM_PUSH:
				flight.request.type = VZ_DANCE;
				light.request.type = FIRE;
				break;
			case FRONT_UP_TILT:
				flight.request.type = FLIP_BEHIND;
				light.request.type = FIRE;
				break;
			case BACK_UP_TILT:
				flight.request.type = FLIP_AHEAD;
				light.request.type = FIRE;
				break;
			case LEFT_UP_TILT:
				flight.request.type = FLIP_RIGHT;
				light.request.type = FIRE;
				break;
			case RIGHT_UP_TILT:
				flight.request.type = FLIP_LEFT;
				light.request.type = FIRE;
				break;
			case FRONT_DOWN_TILT:
				light.request.type = BLINK_ORANGE;
				break;
			case BACK_DOWN_TILT:
				light.request.type = BLINK_ORANGE;
				break;
			case LEFT_DOWN_TILT:
				light.request.type = BLINK_ORANGE;
				break;
			case RIGHT_DOWN_TILT:
				light.request.type = BLINK_ORANGE;
				break;
			case INDETERMINATE:		
				ros::Duration(2).sleep();
		
		 		executed = true;
		 		finished = true;
				return 0;
				break;
		}
	
		bool success = false;
	
		flight.request.duration = 0;
		/*TODO REMOVED FOR GLEE
		//Flight animations
		if(flight.request.type != UNDEFINED_TYPE)
			success |= ros::service::call("ardrone/setflightanimation", flight);
		
		
		light.request.freq = 4;
		light.request.duration = 2;
	
		//LED animations
		if(light.request.type != UNDEFINED_TYPE)
			success |= ros::service::call("ardrone/setledanimation", light);
		
		if(!success){
			ROS_ERROR("Failed to call flight/led animation");
		}
		TODO REMOVED FOR GLEE*/
	    success = true;
	    
		executed = true;
		steadyFlightStart = ros::Time::now();
	}

	return 0;
}

bool AnimationState::isLocked(){ 
	return !finished;
}


//==============================================================================
// Takeoff State
//==============================================================================

/*
	Taking off
*/
TakeoffState::TakeoffState(Messenger* m){
	messenger = m;
}

TakeoffState::~TakeoffState(){
	messenger = 0;
}

void TakeoffState::enter(Navdata n, Twist f){
	std::cout << "Entering TAKEOFF state" << std::endl;
	if(n.state == LANDED){
		steady = false;
		
		//TAKEOFF!
		LedAnim light;
		light.request.type = RED_SNAKE;
		light.request.freq = 2;
		light.request.duration = 2;
		ros::service::call("ardrone/setledanimation", light);
		ros::Duration(2).sleep();
		
		messenger->sendTakeoff();
		
		steadyFlightStart = ros::Time::now();
	}else{
		steady = true;
	}
}

void TakeoffState::exit(Navdata n, Twist f){}

ControlCommand* TakeoffState::perform(Navdata navdata, Twist force){
	//don't leave this state until state is hovering..?
	//questionable, maybe should leave as soon as takeoff is complete
	if(navdata.state == HOVERING){
		if(magnitude(force.linear) > LIN_STABLE || magnitude(force.angular) > ROT_STABLE){
			steadyFlightStart = ros::Time::now();
		}else if((ros::Time::now() - steadyFlightStart).toSec() >= DURATION_STABLE){
			steady = true;
		}
	}
	return 0;
}

bool TakeoffState::isLocked(){ 
	return !steady;
}


//==============================================================================
// Land State
//==============================================================================

/*
	Landing or landed
*/
LandState::LandState(Messenger* m){
	messenger = m;
}

LandState::~LandState(){
	messenger = 0;
}

void LandState::enter(Navdata n, Twist f){
	std::cout << "Entering LAND state" << std::endl;
	
	if(isFlying(n)){
		//Time to land
		locked = true;
		messenger->sendLand();
	}else{
		//not flying, therefore not going to land
		locked = false;
	}
	Duration(5.0).sleep();
}

void LandState::exit(Navdata n, Twist f){}

ControlCommand* LandState::perform(Navdata navdata, Twist force){
	//leave this state once we have landed
	if(navdata.state == LANDED){
		locked = false;
	}
	return 0;
}

bool LandState::isLocked(){ 
	return locked;
}