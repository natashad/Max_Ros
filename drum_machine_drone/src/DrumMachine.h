#ifndef __DRUM_MACHINE__
#define __DRUM_MACHINE__

#include <mephisto/State.h>
#include <string>
#include "ros/ros.h"
#include <ardrone_autonomy/Navdata.h>
#include "geometry_msgs/Twist.h"

using namespace ros;
using namespace geometry_msgs;
using namespace ardrone_autonomy;

enum Instrument {
	NONE = 0,
	BASS_2 = 35,
	BASS_1 = 36,
	SIDESTICK_RIM = 37,
	SNARE_1 = 38,
	CLAP = 39,
	SNARE_2 = 40,
	LOW_TOM_2 = 41,
	C_HI_HAT = 42,
	LOW_TOM_1 = 43,
	P_HI_HAT = 44,
	MID_TOM_2 = 45,
	O_HI_HAT = 46,
	MID_TOM_1 = 47,
	HI_TOM_2 = 48,
	CR_CYMBAL_1 = 49,
	HI_TOM_1 = 50,
	R_CYMBAL_1 = 51,
	CH_CYMBAL_1 = 52,
	RIDE_BELL = 53,
	TAMBOURINE = 54,
	SP_CYMBAL = 55,
	COWBELL = 56,
	CR_CYMBAL_2 = 57,
	VIBRASLAP = 58,
	R_CYMBAL_2 = 59
};

class DrumMachine : public StateMachine {
	protected:
	Publisher osc_pub;
	
	public:
	DrumMachine(Messenger* m);
	~DrumMachine();
	State* getNextState();
};

class HitState : public State {
	Time start;
	bool steady;

	public:
	HitState();
	void enter(Navdata, Twist);
	void exit(Navdata, Twist);
	ControlCommand* perform(Navdata, Twist);
	bool isLocked();
};
extern State* hitState;

#endif