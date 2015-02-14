/*
 * Lifter.h
 *
 *  Created on: Feb 2, 2015
 *      Author: paradigm
 */
#include "WPILib.h"
#include "VacuumSensors.h"
#include "Vacuum.h"

#ifndef SRC_LIFTER_H_
#define SRC_LIFTER_H_

class Lifter {
private:
	bool grabbingTote;

	double toteHeight = 12.0;	//inches
	double floorHeight = 0.0;	//inches
	int currentSetpoint = 0;
	const int vacuumOffCount = 3 * 20;
	int countSinceLastRetract = vacuumOffCount + 1;
	double threshold = 0.1;	//acceptable tolerance in inches
	static const int FLOOR = 0.0;
	static const int TOTE = 12.0;

	CANTalon* lifterMotor;
	DoubleSolenoid* toteGrabber;
	DoubleSolenoid* toteDeployer;
	DoubleSolenoid* vacuumDeployer;
	Vacuum* vacuum1;
	VacuumSensors* vacuumSensors;

public:
	Lifter(double p, double i, double d, CANTalon* lifterMotor,
			DoubleSolenoid* toteGrabber, DoubleSolenoid* toteDeployer,
			DoubleSolenoid* vacuumDeployer, Vacuum* vacuum1, VacuumSensors* vacuumSensors);
	void MoveTo(double setpoint);
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
	bool GrabbingTote();
	virtual ~Lifter();
};

#endif /* SRC_LIFTER_H_ */
