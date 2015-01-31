/*
 * AccelPID.h
 *
 *  Created on: Jan 28, 2015
 *      Author: Programming
 */

#ifndef SRC_ACCELPID_H_
#define SRC_ACCELPID_H_

#include <MecanumDriveTrain.h>
#include "WPILib.h"
#include "Commands/PIDSubsystem.h"

class AccelPID : public PIDSubsystem {
private:
	double p;
	double i;
	double d;

	static const int X = 1;
	static const int Y = 2;
	static const int Z = 3;

	int axis;

	Accelerometer* accel;
	MechanumDriveTrain* driveTrain;
public:
	AccelPID(double p, double i, double d, Accelerometer* accel, MechanumDriveTrain* driveTrain, int axis);
	double ReturnPIDInput();
	void UsePIDOutput(double output);
	void InitDefaultCommand();
	void Enable();
	void SetSetpoint(double setpoint);
	void Reset();
	void Disable();
	virtual ~AccelPID();
};

#endif /* SRC_ACCELPID_H_ */
