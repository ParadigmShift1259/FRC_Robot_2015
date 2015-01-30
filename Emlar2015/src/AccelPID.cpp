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
		MechanumDriveTrain* driveTrain) :
		PIDSubsystem("AccelPID", p, i, d) {
	this->p = p;
	this->i = i;
	this->d = d;
	this->accel = accel;
	this->driveTrain = driveTrain;
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
	driveTrain->SetAccelOffset(output);
}

double AccelPID::ReturnPIDInput() {
	return accel->GetX();
}

void AccelPID::InitDefaultCommand() {

}

AccelPID::~AccelPID() {
	// TODO Auto-generated destructor stub
}

