#ifndef __TERPSICHORE_TRANSLATOR__
#define __TERPSICHORE_TRANSLATOR__

#include "LRReceiver.h"

/**
 * Purpose: take the beat and note information and generate a path
 * 
 */
class Conductor {
	//reference position and its derivatives
	public:
	double x, y, z;
	double x_dot, y_dot, z_dot;
	double x_ddot, y_ddot, z_ddot;
	
	
	Conductor(){};
	void calculatePosition(Transport tr);
};

#endif