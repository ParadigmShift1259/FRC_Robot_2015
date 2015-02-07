/*
 * Lifter.h
 *
 *  Created on: Feb 2, 2015
 *      Author: paradigm
 */
#include "WPILib.h"
#include "Vacuum.h"

#ifndef SRC_LIFTER_H_
#define SRC_LIFTER_H_

class Lifter {
private:
	bool grabbingTote;


	double toteHeight = 12.0;	//inches
	double coopStep = 6.0;	//inches
	double floorHeight = 0.0;	//inches
	double scoringPlatform = 2.0;	//inches
	int currentSetpoint = 0;
	const int vacuumOffCount = 3 * 20;
	int countSinceLastRetract = vacuumOffCount + 1;
	double threshold = 0.1;	//acceptable tolerance in inches
	static const int FLOOR = 0;
	static const int TOTE = 1;
	static const int COOPSTEP = 2;
	static const int COOPSTEP1TOTE = 3;
	static const int COOPSTEP2TOTE = 4;
	static const int COOPSTEP3TOTE = 5;
	static const int SCORINGPLATFORM = 6;
	double levels[7] = { floorHeight, toteHeight, coopStep, coopStep
			+ toteHeight, coopStep + (2 * toteHeight), coopStep + (3 * toteHeight),
			scoringPlatform }; //inches floor,scoring platform, step, step+tote, step+2totes, step+3totes,tote

	CANTalon* lifterMotor;
	DoubleSolenoid* toteGrabber;
	DoubleSolenoid* toteDeployer;
	DoubleSolenoid* vacuumDeployer;
	Vacuum* vacuum1;

public:
	Lifter(double p, double i, double d, CANTalon* lifterMotor,
			DoubleSolenoid* toteGrabber, DoubleSolenoid* toteDeployer,
			DoubleSolenoid* vacuumDeployer, Vacuum* vacuum1);
	void MoveTo(int level);
	void BeginAutoGrabTote();
	void RetractVacuum();
	void DeployTote();
	void RetractTote();
	void GrabTote();
	void ReleaseTote();
	void AutoGrabTote();
	void StartVacuums();
	void StopVacuums();
	bool VacuumsAttached();
	bool InPos();
	virtual ~Lifter();
};

#endif /* SRC_LIFTER_H_ */
