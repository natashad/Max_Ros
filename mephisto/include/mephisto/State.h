#ifndef __AGENTSTATE__
#define __AGENTSTATE__

#include <string>
#include <iostream>
#include "ros/ros.h"
#include <ardrone_autonomy/Navdata.h>
#include <ardrone_autonomy/FlightAnim.h>
#include <ardrone_autonomy/LedAnim.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "Core.h"
#include "ForceEstimator.h"

using namespace ros;
using namespace geometry_msgs;
using namespace ardrone_autonomy;

//==============================================================================
// Constants
//==============================================================================

static const double LIN_THRESHOLD = 0.38;
static const double ROT_THRESHOLD = 1300.0;
static const double YAW_THRESHOLD = 1300.0;

static const double LIN_STABLE = 0.2;
static const double ROT_STABLE = 900.0;

static const double DURATION_STABLE = 2.0;
static const double DOUBLE_TAP_TIME = 3.0;

static const double FACTOR = sqrt(3.0);
static const double g = 9.8;

enum FlightAnimation  {
	PHI_M30_DEG,
	PHI_30_DEG,
	THETA_M30_DEG,
	THETA_30_DEG,
	THETA_20DEG_YAW_200DEG,
	THETA_20DEG_YAW_M200DEG,
	TURNAROUND,
	TURNAROUND_GODOWN,
	YAW_SHAKE,
	YAW_DANCE,
	PHI_DANCE,
	THETA_DANCE,
	VZ_DANCE,
	WAVE,
	PHI_THETA_MIXED,
	DOUBLE_PHI_THETA_MIXED,
	FLIP_AHEAD,
	FLIP_BEHIND,
	FLIP_LEFT,
	FLIP_RIGHT
};

#define UNDEFINED_TYPE		100

enum LEDAnimation {
	BLINK_GREEN_RED,
	BLINK_GREEN,
	BLINK_RED,
	BLINK_ORANGE,
	SNAKE_GREEN_RED,
	FIRE,
	STANDARD,
	RED,
	GREEN,
	RED_SNAKE,
	BLANK,
	LEFT_GREEN_RIGHT_RED,
	LEFT_RED_RIGHT_GREEN,
	BLINK_STANDARD
};

enum ForceType {
	INDETERMINATE,		//0
	FRONT_PUSH,			//1
	BACK_PUSH,			//2
	LEFT_PUSH,			//3
	RIGHT_PUSH,			//4
	TOP_PUSH,			//5
	BOTTOM_PUSH,		//6
	ECCENTRIC_PUSH,		//7
	FRONT_UP_TILT,		//8
	FRONT_DOWN_TILT,	//9
	BACK_UP_TILT,		//10
	BACK_DOWN_TILT,		//11
	LEFT_UP_TILT,		//12
	LEFT_DOWN_TILT,		//13
	RIGHT_UP_TILT,		//14
	RIGHT_DOWN_TILT,	//15
	TWIST_CCW,			//16
	TWIST_CW			//17
};

//drone state as reported by navdata
enum DroneState {
	UNKNOWN,		//0
	IGNITED,		//1
	LANDED,			//2
	FLYING3,		//3
	HOVERING,		//4
	FLYING_TEST,	//5
	TAKING_OFF,		//6
	FLYING7,		//7
	LANDING,		//8
	LOOPING			//9
};

//==============================================================================
// Class Definitions
//==============================================================================

class State {
	public:
	
	virtual void enter(Navdata, Twist) = 0;
	virtual void exit(Navdata, Twist) = 0;
	virtual ControlCommand* perform(Navdata, Twist) = 0;
	virtual bool isLocked() = 0;
};

//------------------------------------------------------------------------------

class StateMachine : public NavdataListener {
	protected:
	State* curState;
	Navdata curNavdata;
	Twist curForce;
	Messenger* messenger;
	DroneState droneState;
	NodeHandle nh_;
	
	Subscriber force_sub;
	string force_sub_channel;
	
	int mode;
	Twist prevForce;
	
	bool allowTakeoff;
	
	void init(Messenger*, bool);
	
	public:
	
	StateMachine(Messenger*);
	StateMachine(Messenger*, bool);
	~StateMachine();
	
	void checkStateChange();
	virtual State* getNextState();
	void execute();
	void navdataCb(const NavdataConstPtr);
	void forceCb(const geometry_msgs::Twist::ConstPtr&);
};

//------------------------------------------------------------------------------

class HoverState : public State {
	private:
	//For peak detection
	Twist prevForce;
	bool cmdsent;

	public:
	void enter(Navdata, Twist);
	void exit(Navdata, Twist);
	ControlCommand* perform(Navdata, Twist);
	bool isLocked();
};
extern State* hoverState;

//------------------------------------------------------------------------------

class PassiveState : public State {
	Time start;
	Time steadyFlightStart;
	Twist prevForce;

	public:
	void enter(Navdata, Twist);
	void exit(Navdata, Twist);
	ControlCommand* perform(Navdata, Twist);
	bool isLocked();
	
	int stage;
	//This state stores information about the next state to go to
	State* fromState;
	State* toState;
	ForceType triggerType;
};
extern State* passiveState;

//------------------------------------------------------------------------------

class FloatState : public State {
	public:
	void enter(Navdata, Twist);
	void exit(Navdata, Twist);
	ControlCommand* perform(Navdata, Twist);
	bool isLocked();
};
extern State* floatState;

//------------------------------------------------------------------------------

class MoveState : public State {
	Time start;
	Time steadyFlightStart;
	bool steady;

	public:
	void enter(Navdata, Twist);
	void exit(Navdata, Twist);
	ControlCommand* perform(Navdata, Twist);
	bool isLocked();
	Twist trigger;
};
extern State* moveState;

//------------------------------------------------------------------------------

class AnimationState : public State {
	Twist initialForce;
	bool executed;
	Time steadyFlightStart;
	bool finished;

	public:
	AnimationState();
	void enter(Navdata, Twist);
	void exit(Navdata, Twist);
	ControlCommand* perform(Navdata, Twist);
	bool isLocked();
};
extern State* animState;

//------------------------------------------------------------------------------

class TakeoffState : public State {
	Messenger* messenger;
	Time steadyFlightStart;
	bool steady;

	public:
	TakeoffState(Messenger*);
	~TakeoffState();
	void enter(Navdata, Twist);
	void exit(Navdata, Twist);
	ControlCommand* perform(Navdata, Twist);
	bool isLocked();
};
extern State* takeoffState;

//------------------------------------------------------------------------------

class LandState : public State {
	Messenger* messenger;
	bool locked;

	public:
	LandState(Messenger*);
	~LandState();
	void enter(Navdata, Twist);
	void exit(Navdata, Twist);
	ControlCommand* perform(Navdata, Twist);
	bool isLocked();
};
extern State* landState;

//==============================================================================
// Function Headers for utility functions
//==============================================================================

void blink_green();
void blink_red();
void blink_orange();
double magnitude(Vector3);
double xymagnitude(Vector3);
double xyangle(Vector3);
bool isFlying(Navdata);
ForceType getForceType(Twist);

#endif