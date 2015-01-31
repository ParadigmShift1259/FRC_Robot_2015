#ifndef GyroPID_H
#define GyroPID_H

#include <MecanumDriveTrain.h>
#include "Commands/PIDSubsystem.h"
#include "WPILib.h"
#include "CorrectedGyro.h"

class GyroPID: public PIDSubsystem
{
private:
	CorrectedGyro* roboGyro;
	MechanumDriveTrain* driveTrain;
public:
	GyroPID(double p, double i, double d, CorrectedGyro* roboGyro, MechanumDriveTrain* driveTrain);
	double ReturnPIDInput();
	void UsePIDOutput(double output);
	void SetPIDValues(double p,double i,double d);
	void InitDefaultCommand();
	void SetSetpoint(double setpoint);
	void Reset();
};

#endif
