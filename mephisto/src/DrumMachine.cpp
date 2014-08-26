#include "State.h"
#include "DrumMachine.h"
#include <std_msgs/Float64MultiArray.h>
#include <cmath>

DrumMachine::DrumMachine(Messenger* m) : StateMachine(m){
	osc_pub = nh_.advertise<std_msgs::Float64MultiArray>("to_ableton",  100);
	curState = hoverState;
	hitState = new HitState();
}

DrumMachine::~DrumMachine(){
}

State* DrumMachine::getNextState(){
	if(curState->isLocked()){
		return curState;
	}else if(curState == hoverState){
		//only detect hits if the drone is either landed or hovering, not flying nor taking off
		if(droneState == LANDED || droneState == HOVERING){
			//See if the force is big enough to trigger an animation or other state transitions
			if(magnitude(curForce.linear) > LIN_THRESHOLD || magnitude(curForce.angular) > ROT_THRESHOLD){
				//Look for a peak in the force data
				if(magnitude(curForce.linear) < magnitude(prevForce.linear) || 
					magnitude(curForce.angular) < magnitude(prevForce.angular)){
					//peak found at prev force;		
					ForceType type = getForceType(prevForce);
					int pitch = NONE;
					float duration = 0.25;
				
					double greater = fmax((magnitude(curForce.linear) - LIN_THRESHOLD)/LIN_THRESHOLD, (magnitude(curForce.angular) - ROT_THRESHOLD)/ROT_THRESHOLD);
				
					int velocity = (int) fmin(127.0*(greater + 0.3), 127.0);
			
					switch(type){
					case TWIST_CCW:
						pitch = VIBRASLAP;
						break;
					case TWIST_CW:
						pitch = COWBELL;
						break;
					case ECCENTRIC_PUSH:
						pitch = C_HI_HAT;
						break;
					case FRONT_PUSH:
						pitch = SNARE_1;
						break;
					case BACK_PUSH:
						pitch = HI_TOM_1;
						break;
					case LEFT_PUSH:
						pitch = LOW_TOM_1;
						break;
					case RIGHT_PUSH:
						pitch = MID_TOM_1;
						break;
					case TOP_PUSH:
						pitch = BASS_1;
						break;
					case BOTTOM_PUSH:
						pitch = BASS_1;
						break;
					case FRONT_UP_TILT:
						pitch = C_HI_HAT;
						break;
					case BACK_UP_TILT:
						pitch = C_HI_HAT;
						break;
					case LEFT_UP_TILT:
						pitch = R_CYMBAL_1;
						break;
					case RIGHT_UP_TILT:
						pitch = CR_CYMBAL_1;
						break;
					case FRONT_DOWN_TILT:
						pitch = C_HI_HAT;
						break;
					case BACK_DOWN_TILT:
						pitch = C_HI_HAT;
						break;
					case LEFT_DOWN_TILT:
						pitch = P_HI_HAT;
						break;
					case RIGHT_DOWN_TILT:
						pitch = O_HI_HAT;
						break;
					case INDETERMINATE:		
					default:
						break;
					}

					std_msgs::Float64MultiArray msg;
					std::vector<double> data;
					data.push_back((double)pitch);
					data.push_back(duration);
					data.push_back((double)velocity);
					msg.data = data;
				
					std::cout << " note: " << pitch << ", " << duration << ", " << velocity << std::endl;
				
					osc_pub.publish(msg);
			
					return hitState;
				}
			}
		}

		return hoverState;
	}else if(curState == hitState){
		return hoverState;
	}else{
		return curState;
	}
}


//==============================================================================
// Hit State
//==============================================================================

HitState::HitState(){
}

void HitState::enter(Navdata nav, Twist f){
	std::cout << "Entering HIT state" << std::endl;
	blink_red();
	steady = false;
	start = ros::Time::now();
}

void HitState::exit(Navdata n, Twist f){}

ControlCommand* HitState::perform(Navdata navdata, Twist force){
	//Hover command
	if(magnitude(force.linear) > LIN_THRESHOLD || magnitude(force.angular) > ROT_THRESHOLD){
		start = ros::Time::now();
	}else if((ros::Time::now() - start).toSec() >= 0.25){
		steady = true;
	}
	
    return new ControlCommand(0, 0, 0, 0);
}

bool HitState::isLocked(){ 
	return !steady;
}

State* hitState;