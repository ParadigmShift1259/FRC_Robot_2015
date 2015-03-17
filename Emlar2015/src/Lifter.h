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

	double liftercpr = 120.0 * 4.0;
	double lifterGearRatio = 1.0;
	double lifterSprocketDiameter = 2.406;
	double lifterInchesPerRotation = pi * lifterSprocketDiameter
			* lifterGearRatio;
	double lifterInchesPerClick = lifterInchesPerRotation / liftercpr;

	double toteHeight = 12.0 * lifterInchesPerRotation;	//inches
	double floorHeight = 0.0 * lifterInchesPerRotation;	//inches
	int currentSetpoint = 0;
	const int vacuumOffCount = 3 * 20;
	int countSinceLastRetract = vacuumOffCount + 1;
	int threshold = 0.125 / lifterInchesPerClick;//acceptable tolerance in inches
	int FLOOR = 0.0;
	const int settleOffCount = 1*20;
	int settleWait = settleOffCount+1;
	const int grabOffCount = 1*20;
	int grabWait = grabOffCount;

	int VACUUMCLEARANCE = 15.0 / lifterInchesPerClick;
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
	void Disable();
	bool Zeroed();
	bool SafeChangeVacuumState();
	void StartEmergencyClear();
	bool VacuumsAttached();
	bool InPos();
	bool GrabbingTote();
	virtual ~Lifter();
};

#endif /* SRC_LIFTER_H_ */
