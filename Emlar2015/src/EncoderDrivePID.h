/*
 * AccelPID.h
 *
 *  Created on: Jan 28, 2015
 *      Author: Programming
 */

#ifndef SRC_ENCODERDRIVEPID_H_
#define SRC_ENCODERDRIVEPID_H_

#include <MecanumDriveTrain.h>
#include "WPILib.h"
#include "Commands/PIDSubsystem.h"
#include "DriveEncoders.h"

class EncoderDrivePID : public PIDSubsystem {
private:
	double p;
	double i;
	double d;

	static const int STRAIGHT = 1;
	static const int STRAIF = 2;

	int axis;

	DriveEncoders* driveEncoders;
	MechanumDriveTrain* driveTrain;
public:
	EncoderDrivePID(double p, double i, double d, DriveEncoders* driveEncoders, MechanumDriveTrain* driveTrain, int axis);
	double ReturnPIDInput();
	void UsePIDOutput(double output);
	void InitDefaultCommand();
	void Enable();
	void SetSetpoint(double setpoint);
	void Reset();
	void Disable();
	virtual ~EncoderDrivePID();
};

#endif /* SRC_ENCODERDRIVEPID_H_ */
