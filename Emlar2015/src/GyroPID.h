#ifndef GyroPID_H
#define GyroPID_H

#include "Commands/PIDSubsystem.h"
#include "WPILib.h"
#include "MechanumDriveTrain.h"

class GyroPID: public PIDSubsystem
{
private:
	Gyro* roboGyro;
	MechanumDriveTrain* driveTrain;
public:
	GyroPID(double p, double i, double d, Gyro* roboGyro, MechanumDriveTrain* driveTrain);
	double ReturnPIDInput();
	void UsePIDOutput(double output);
	void InitDefaultCommand();
	void SetSetpoint(double setpoint);
};

#endif
