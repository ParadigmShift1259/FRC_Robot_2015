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
	bool lowered = false;
	bool skipVacuumSensors = false;
	bool justStarted = false;
	bool cleared = false;
	bool clearing = false;

	const double pi =
			3.141592653589793238462643383279502884197169399375105820974944592307816406286;

	double liftercpr = 120.0 * 4.0;
	double lifterGearRatio = 1.0;
	double lifterSprocketDiameter = 2.406;
	double lifterInchesPerRotation = pi * lifterSprocketDiameter
			* lifterGearRatio;
	double lifterInchesPerClick = lifterInchesPerRotation / liftercpr;

	int DROPDISTANCE = 4.0 / lifterInchesPerClick;
	int VACUUMCLEARANCE = 14.0 / lifterInchesPerClick;
	int FLOOR = 0.0;
	int THRESHOLD = 0.125 / lifterInchesPerClick; //acceptable tolerance in inches

	int CLEARANCE = 2.0 / lifterInchesPerClick;
	int SCORINGPLATFORM = 2.0 / lifterInchesPerClick;
	int COOPSTEP = 6.25 / lifterInchesPerClick;
	int TOTE = 12.0 / lifterInchesPerClick; //make sure to change the ones in the lifter class

	int VACUUM_RETRACT_TIME = 1 * 20;
	int GRABBER_RETRACT_TIME = 1 * 20;
	int ARM_DEPLOY_TIME = 2 * 20;
	int GRABBER_EXTEND_TIME = 1 * 20;
	int SETTLE_TIME = 1 * 20;

	int countSinceVacuumRetractTriggered = VACUUM_RETRACT_TIME + 1;
	int countSinceToteGrabberRetractTriggered = GRABBER_RETRACT_TIME + 1;
	int countSinceToteGrabberExtendTriggered = GRABBER_EXTEND_TIME + 1;
	int countSinceArmDeployTriggered = ARM_DEPLOY_TIME +1;
	int countSinceSettleTriggered = SETTLE_TIME +1;

	int dropCount = 0;
	int emerClearCount = 0;

	CANTalon* lifterMotor;
	DoubleSolenoid* toteGrabber;
	DoubleSolenoid* toteDeployer;
	DoubleSolenoid* vacuumDeployer;
	Vacuum** vacuums;
	VacuumSensors* vacuumSensors;
	IntakeWheels* intakeWheels;

	int numberOfVacuums;
protected:

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
	void SkipVacuumSensors();

	void StartVacuums();
	void StopVacuums();
	void Drop();

	void LifterQueuedFunctions();
	void Zero();
	void Disable();
	void Clear();

	bool Zeroed();
	bool Cleared();
	bool Clearing();
	bool SafeToChangeVacuumState();

	void StartEmergencyClear();

	bool VacuumsAttached();
	bool InPos();
	bool GrabbingTote();
	bool DroppingTote();
	bool JustStarted();

	void EndStartPeriod();

	int GetDROPDISTANCE() {return DROPDISTANCE;}
	int GetVACUUMCLEARANCE() {return VACUUMCLEARANCE();}
	int GetFLOOR() {return FLOOR;}
	int GetCLEARANCE() {return CLEARANCE;}
	int GetSCORINGPLATFORM() {return SCORINGPLATFORM;}
	int GetCOOPSTEP() {return COOPSTEP;}
	int GetTOTE() {return TOTE;}
	double GetHeight() {return (lifterMotor->GetPosition() * lifterInchesPerClick);}
	double GetEncPosition() {return (((double) lifterMotor->GetEncPosition()) / lifterInchesPerClick);}
	double GetSetpoint() {return (((double) lifterMotor->GetSetpoint()) / lifterInchesPerClick);}
	double Getliftercpr() {return liftercpr;}

	virtual ~Lifter();
};

#endif /* SRC_LIFTER_H_ */
