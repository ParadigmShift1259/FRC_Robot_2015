/*
 * Lifter.h
 *
 *  Created on: Feb 2, 2015
 *      Author: paradigm
 */
#include "WPILib.h"

#ifndef SRC_LIFTER_H_
#define SRC_LIFTER_H_

class Lifter {
private:
	double toteHeight = 12.0;	//inches
	double coopStep = 6.0;	//inches
	double floorHeight = 0.0;	//inches
	double scoringPlatform = 2.0;	//inches
	int currentSetpoint = 0;
	double threshold = 0.1;	//acceptable tolerance in inches
	CANTalon* lifterMotor;
	double levels[7] = { floorHeight, toteHeight, coopStep, coopStep
			+ toteHeight, coopStep + 2 * toteHeight, coopStep + 3 * toteHeight,
			scoringPlatform }; //inches floor,scoring platform, step, step+tote, step+2totes, step+3totes,tote
public:
	Lifter(double p, double i, double d, CANTalon* lifterMotor);
	void MoveTo(int level);
	bool InPos();
	virtual ~Lifter();
};

#endif /* SRC_LIFTER_H_ */
