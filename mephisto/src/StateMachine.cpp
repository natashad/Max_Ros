#include "State.h"
#include <cmath>

//==============================================================================
// Utility Functions
//==============================================================================

double magnitude(Vector3 f){
	return sqrt(f.x*f.x + f.y*f.y + f.z*f.z);
}

double xymagnitude(Vector3 f){
	return sqrt(f.x*f.x + f.y*f.y);
}

double xyangle(Vector3 f){
	return atan2(f.y, f.x);
}

bool isFlying(Navdata n){
	if(	n.state == FLYING3 ||
		n.state == HOVERING ||
		n.state == FLYING_TEST ||
		n.state == LOOPING){
		return true;
	}else{
		return false;
	}
}

ForceType getForceType(Twist force){
	ForceType type = INDETERMINATE;

	//Decide what animation to trigger based on the nature of the force applied
	//First look at rotations about the z axis. If these are big then we can say
	//that the interaction was a twist, or a very eccentric push. We can look
	//at the accelerations in the x and y directions to find this out
	if(abs(force.angular.z) > ROT_THRESHOLD / FACTOR){
		if(abs(force.linear.x) > LIN_THRESHOLD / FACTOR ||
			abs(force.linear.y) > LIN_THRESHOLD / FACTOR){
			//This is an eccentric push
			type = ECCENTRIC_PUSH;		
		}else{
			//This is a yaw twist
			if(force.angular.z > 0.0){
				type = TWIST_CCW;
			}else{
				type = TWIST_CW;
			}
		}
	}
	//Next look at changes in linear acceleration in x and y
	//If these numbers are big then it's fairly safe to say the interaction
	//was a push from one of the sides
	else if(abs(force.linear.x) > LIN_THRESHOLD / FACTOR ||
			abs(force.linear.y) > LIN_THRESHOLD / FACTOR){
		if(abs(force.linear.x) > LIN_THRESHOLD / FACTOR){
			//front/back
			if(force.linear.x < 0.0){
				type = FRONT_PUSH;
			}else{
				type = BACK_PUSH;
			}
		}else{
			//left/right
			if(force.linear.y < 0.0){
				type = LEFT_PUSH;
			}else{
				type = RIGHT_PUSH;
			}
		}
	}
	//Then we look at rotations about the x and y axis. Large numbers in these
	//guys result from a tilt. In addition, look at acceleration in the z 
	//direction to determine whether it was a tilt up or down
	else if(abs(force.angular.x) > ROT_THRESHOLD / FACTOR ||
			abs(force.angular.y) > ROT_THRESHOLD / FACTOR){
		if(force.linear.z > 0.0){
			//tilt up
			if(abs(force.angular.x) > abs(force.angular.y)){
				//tilt from the left or right
				if(force.angular.x > 0.0){
					type = LEFT_UP_TILT;
				}else{
					type = RIGHT_UP_TILT;
				}
			}else{
				//tilt from the front or back
				if(force.angular.y > 0.0){
					type = BACK_UP_TILT;
				}else{
					type = FRONT_UP_TILT;
				}
			}
		}else{
			//tilt down
			if(abs(force.angular.x) > abs(force.angular.y)){
				//tilt from the left or right
				if(force.angular.x > 0.0){
					type = LEFT_DOWN_TILT;
				}else{
					type = RIGHT_DOWN_TILT;
				}
			}else{
				//tilt from the front or back
				if(force.angular.y > 0.0){
					type = BACK_DOWN_TILT;
				}else{
					type = FRONT_DOWN_TILT;
				}
			}
		}
	
	}
	//Finally we check the z acceleration to see if there was a push up or down
	else if(abs(force.linear.z) > LIN_THRESHOLD / FACTOR ){
		if(force.linear.z > 0.0){
			type = BOTTOM_PUSH;
		}else{
			type = TOP_PUSH;
		}
	}
	//else.....didn't understand the command....?
	else{
		std::cout << "Could not interpret force" << std::endl;
		std::cout << "Linear\tx:" << force.linear.x << "\ty:" << force.linear.y 
						<< "\tz:" << force.linear.z << std::endl;
		std::cout << "Angular\tx:" << force.angular.x << "\ty:" << force.angular.y 
						<< "\tz:" << force.angular.z << std::endl;
		type = INDETERMINATE;
	}
	
	
	return type;
}

void blink_green(){
	LedAnim light;
	light.request.type = BLINK_GREEN;
	light.request.freq = 3;
	light.request.duration = 1;
	ros::service::call("ardrone/setledanimation", light);
}

void blink_red(){
	LedAnim light;
	light.request.type = BLINK_RED;
	light.request.freq = 3;
	light.request.duration = 1;
	ros::service::call("ardrone/setledanimation", light);
}

void blink_orange(){
	LedAnim light;
	light.request.type = BLINK_ORANGE;
	light.request.freq = 3;
	light.request.duration = 1;
	ros::service::call("ardrone/setledanimation", light);
}

//==============================================================================
// State Machine Class
//==============================================================================

StateMachine::StateMachine(Messenger* m) : curNavdata(), curForce(), prevForce()
{
	init(m, false);
}

StateMachine::StateMachine(Messenger* m, bool t) : curNavdata(), curForce(), prevForce()
{
	init(m, t);
}

void StateMachine::init(Messenger* m, bool takeoffAllowed){
	this->messenger = m;
	messenger->addNavdataListener(this);
	force_sub_channel = nh_.resolveName("ardrone/extForce");
	force_sub = nh_.subscribe(force_sub_channel, 10, &StateMachine::forceCb, this);
	
	hoverState = new HoverState();
	floatState = new FloatState();
	passiveState = new PassiveState();
	animState = new AnimationState();
	takeoffState = new TakeoffState(m);
	landState = new LandState(m);
	moveState = new MoveState();
	
	allowTakeoff = takeoffAllowed;
	if(takeoffAllowed){/*
	    switch(droneState){
	        case UNKNOWN:
            case LANDED:
            case LANDING:
            case TAKING_OFF:
                curState = landState;
            case IGNITED:
            case FLYING3:
            case HOVERING:
            case FLYING_TEST:
            case FLYING7:
            case LOOPING:
                curState = hoverState;
	    }*/
	    curState = landState;
	}else
		curState = floatState;
}

StateMachine::~StateMachine(){
	messenger = 0;
	delete hoverState; 
	delete floatState;
	delete passiveState;
	delete animState;
	delete takeoffState;
	delete landState;
	delete moveState;
}

void StateMachine::navdataCb(const NavdataConstPtr navdataPtr){
	
	curNavdata = *(navdataPtr.get());
	droneState = (DroneState)curNavdata.state;
}

/**
* callback function for receiving force estimate, should also make state switch decisions
*/
void StateMachine::forceCb(const geometry_msgs::Twist::ConstPtr& forcePtr){
	prevForce = curForce;
	curForce = *(forcePtr.get());
}

/*
	This is called all the time in the main loop. It should do everything,
	basically.
*/
void StateMachine::execute(){
	checkStateChange();
	
	//do what the current state requires
	ControlCommand* c = curState->perform(curNavdata, curForce);
	if(c != 0){
		messenger->sendControlToDrone(*c);
		delete c;
	}
}

/*
	This is the state transition function. Figure out what state to be in next
	based on current information.
*/
State* StateMachine::getNextState(){
	//check for state switch if the current state is not locked
	if(!curState->isLocked()){
		if(curState == hoverState){
			//See if the force is big enough to trigger an animation or other state transitions
			if(magnitude(curForce.linear) > LIN_THRESHOLD || magnitude(curForce.angular) > ROT_THRESHOLD){
				//Look for a peak in the force data
				if(magnitude(curForce.linear) < magnitude(prevForce.linear) || 
					magnitude(curForce.angular) < magnitude(prevForce.angular)){
					//peak found at prev force;		
					ForceType type = getForceType(prevForce);
					
					//A top push is a land command
					if(type == TOP_PUSH){
						return landState;
					}
		    /*TODO REMOVED FOR GLEE
					//Downward tilts initiates double tap to switch modes
					else if(type == LEFT_DOWN_TILT || type == RIGHT_DOWN_TILT ||
							type == FRONT_DOWN_TILT || type == BACK_DOWN_TILT)
					{
						((PassiveState*)passiveState)->fromState = curState;
						((PassiveState*)passiveState)->toState = floatState;
						((PassiveState*)passiveState)->triggerType = type;
						return passiveState;
					}
			TODO REMOVED FOR GLEE*/
					//everything else is flight animations
					else{
						curForce = prevForce;
						return animState;
					}
					
				}else{
					return hoverState;
				}
			}else{
				return hoverState;
			}
		}else if(curState == floatState){
			//See if the force is big enough to trigger a mode toggle
			if(magnitude(curForce.linear) > LIN_THRESHOLD || magnitude(curForce.angular) > ROT_THRESHOLD){
				//look for peaks
				if(magnitude(curForce.linear) < magnitude(prevForce.linear) ||
					magnitude(curForce.angular) < magnitude(prevForce.angular)){
				
					ForceType type = getForceType(prevForce);
					
					//A top push is a land command
					if(type == TOP_PUSH){
						return landState;
					}
					//Downward tilts initiates double tap to switch modes
					if(type == LEFT_DOWN_TILT || type == RIGHT_DOWN_TILT ||
							type == FRONT_DOWN_TILT || type == BACK_DOWN_TILT)
					{
						((PassiveState*)passiveState)->fromState = curState;
						((PassiveState*)passiveState)->toState = hoverState;
						((PassiveState*)passiveState)->triggerType = type;
						return passiveState;
					}
					else if(type == FRONT_PUSH	|| type == BACK_PUSH || type == LEFT_PUSH || type == RIGHT_PUSH){
						((MoveState*)moveState)->trigger = prevForce;
						return moveState;
					}	
				}
			}
			return floatState;
		}else if(curState == passiveState){
			if(((PassiveState*)passiveState)->stage >= 3)
				return ((PassiveState*)passiveState)->toState;
			else
				return ((PassiveState*)passiveState)->fromState;
		}else if(curState == animState){
			return hoverState;
		}else if(curState == takeoffState){
			return hoverState;
		}else if(curState == landState){
			//See if the force is big enough to trigger a takeoff
			if(magnitude(curForce.linear) > LIN_THRESHOLD || magnitude(curForce.angular) > ROT_THRESHOLD){
				//look for peaks
				if(magnitude(curForce.linear) < magnitude(prevForce.linear) ||
					magnitude(curForce.angular) < magnitude(prevForce.angular)){
				
					ForceType type = getForceType(prevForce);
					if(type == FRONT_UP_TILT){
						std::cout << "allowTakeoff: " << allowTakeoff << std::endl;
						//if not flying, then a front up tilts means take off
						if(allowTakeoff){
							return takeoffState;
						}
					}
				}
			}
			//else
			return landState;
		}else if(curState == moveState){
			//must be unlocked at this point
			return floatState;
		}else{
			return curState;
		}
	}
	//if locked, don't switch
	else{
		return curState;
	}
}

void StateMachine::checkStateChange(){
	State* nextState = getNextState();
	//If we should go into a new state
	if(curState != nextState){
		curState->exit(curNavdata, curForce);
		curState = nextState;
		curState->enter(curNavdata, curForce);
	}
}

State* hoverState;
State* floatState;
State* passiveState;
State* animState;
State* takeoffState;
State* landState;
State* moveState;