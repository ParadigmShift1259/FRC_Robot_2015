/*
 * AccelPID.cpp
 *
 *  Created on: Jan 28, 2015
 *      Author: Programming
 */

#include "AccelPID.h"
#include "SmartDashboard/SmartDashboard.h"
#include "LiveWindow/LiveWindow.h"

AccelPID::AccelPID(double p, double i, double d, Accelerometer* accel,
		MechanumDriveTrain* driveTrain, int axis) : //axis: 1=x, 2=y, 3=z
		PIDSubsystem("AccelPID", p, i, d) {
	this->p = p;
	this->i = i;
	this->d = d;
	this->accel = accel;
	this->driveTrain = driveTrain;
	this->axis = axis;
	SetSetpoint(0.0);
	Enable();

	// TODO Auto-generated constructor stub
}

void AccelPID::Enable() {
	GetPIDController()->Enable();
}
void AccelPID::SetSetpoint(double setpoint) {
	GetPIDController()->SetSetpoint(setpoint);
}

void AccelPID::Reset() {
	GetPIDController()->Reset();
}

void AccelPID::UsePIDOutput(double output) {
	switch (axis) {
		case X:
			driveTrain->SetXAccelOffset(output);
			break;
		case Y:
			driveTrain->SetYAccelOffset(output);
			break;
		case Z:
			driveTrain->SetZAccelOffset(output);
			break;
		}
}

double AccelPID::ReturnPIDInput() {
	double input = 0;
	switch (axis) {
	case X:
		input = accel->GetX();
		break;
	case Y:
		input = accel->GetY();
		break;
	case Z:
		input = accel->GetZ();
		break;
	}
	return input;
}

void AccelPID::InitDefaultCommand() {

}

void AccelPID::Disable() {
	GetPIDController()->Disable();
}

AccelPID::~AccelPID() {
	// TODO Auto-generated destructor stub
}

