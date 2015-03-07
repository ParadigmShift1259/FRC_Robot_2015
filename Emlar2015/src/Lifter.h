/*
 * Lifter.h
 *
 *  Created on: Feb 2, 2015
 *      Author: paradigm
 */
#include "WPILib.h"
#include "VacuumSensors.h"
#include "Vacuum.h"
#include "IntakeWheels.h"

#ifndef SRC_LIFTER_H_
#define SRC_LIFTER_H_

class Lifter {
private:
	bool grabbingTote = false;
	bool dropping = false;
	bool emergencyClearing = false;
	bool zeroed = false;

	const double pi =
			3.141592653589793238462643383279502884197169399375105820974944592307816406286;

	const double liftercpr = 120.0;
	const double lifterGearRatio = 1.0;
	const double lifterSprocketDiameter = 1.0;
	const double lifterRotationsPerInch = pi * lifterSprocketDiameter
			* lifterGearRatio;
	const double lifterInchesPerClick = lifterRotationsPerInch / liftercpr;

	double toteHeight = 12.0 * lifterRotationsPerInch;	//inches
	double floorHeight = 0.0 * lifterRotationsPerInch;	//inches
	int currentSetpoint = 0;
	const int vacuumOffCount = 3 * 20;
	int countSinceLastRetract = vacuumOffCount + 1;
	double threshold = 0.1 / lifterRotationsPerInch;//acceptable tolerance in inches
	static const int FLOOR = 0.0;
	static const int TOTE = 12.0;
	double dropCount = 0;
	int numberOfVacuums;
	int clearCount = 0;

	CANTalon* lifterMotor;
	DoubleSolenoid* toteGrabber;
	DoubleSolenoid* toteDeployer;
	DoubleSolenoid* vacuumDeployer;
	Vacuum** vacuums;
	VacuumSensors* vacuumSensors;
	IntakeWheels* intakeWheels;

public:
	Lifter(double p, double i, double d, CANTalon* lifterMotor,
			DoubleSolenoid* toteGrabber, DoubleSolenoid* toteDeployer,
			DoubleSolenoid* vacuumDeployer, Vacuum** vacuums,
			VacuumSensors* vacuumSensors, IntakeWheels* intakeWheels,
			int numberOfVacuums);
	void MoveTo(double setpoint);
	void BeginAutoGrabTote();
	void DeployVacuum();
	void RetractVacuum();
	void DeployTote();
	void RetractTote();
	void ContinueDrop();
	void GrabTote();
	void ReleaseTote();
	void AutoGrabTote();
	void StartVacuums();
	void StopVacuums();
	void Drop();
	void LifterQueuedFunctions();
	void Zero();
	bool Zeroed();
	void StartEmergencyClear();
	bool VacuumsAttached();
	bool InPos();
	bool GrabbingTote();
	virtual ~Lifter();
};

#endif /* SRC_LIFTER_H_ */
