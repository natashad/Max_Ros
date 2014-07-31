#include "State.h"
#include <cmath>

static const double omega = 1.5;
static const double Kr = 5.0;
static const double Kc = -0.01 / g;

//==============================================================================
// Float State
//==============================================================================

/*
	Response to a linear push in the planar directions. 
*/
void FloatState::enter(Navdata n, Twist f){
	std::cout << "Entering FLOAT state" << std::endl;
	blink_red();
}

void FloatState::exit(Navdata n, Twist f){}

ControlCommand* FloatState::perform(Navdata navdata, Twist force){
	
	//TODO 

	//Hover controller off
	return new ControlCommand(0,0,0,0, true);
}

bool FloatState::isLocked(){ 
	return false; //TODO
}

//==============================================================================
// Move State
//==============================================================================

/*
	Response to a push in the planar directions. 
*/
void MoveState::enter(Navdata n, Twist f){
	std::cout << "Entering MOVE state" << std::endl;
	blink_red();
	
	start = Time::now();
	steadyFlightStart = start;
	steady = false;
}

void MoveState::exit(Navdata n, Twist f){}

ControlCommand* MoveState::perform(Navdata navdata, Twist force){
	
	Time now = Time::now();
	
	if(magnitude(force.linear) > LIN_STABLE || magnitude(force.angular) > ROT_STABLE){
		steadyFlightStart = Time::now();
	}else if((now - steadyFlightStart).sec >= DURATION_STABLE){
		steady = true;
	}
	
	double t2 = now.sec + now.nsec/1000000000.0;
	double t1 = start.sec + start.nsec/1000000000.0;
	double t = t2 - t1;
	
	double F = xymagnitude(trigger.linear);
	double angle = xyangle(trigger.linear);
	
	double Fx = F*cos(angle);
	double Fy = F*sin(angle);
	
	double vx = navdata.vx;
	double vy = navdata.vy;
	
	double ux = -(Fx*Kr*omega*omega*exp(-omega*t)*(omega*t - 1.0))/g;// + Kc*(vx - Fx*Kr*omega*omega*t*exp(-omega*t));
	double uy = -(Fy*Kr*omega*omega*exp(-omega*t)*(omega*t - 1.0))/g;// + Kc*(vy - Fy*Kr*omega*omega*t*exp(-omega*t));
	
	//Hover controller off
	return new ControlCommand(uy, -ux, 0, 0, false);
}

bool MoveState::isLocked(){ 
	return !steady;
}