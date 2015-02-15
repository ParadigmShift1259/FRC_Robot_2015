/*
 * EncoderDrivePID.cpp
 *
 *  Created on: Jan 28, 2015
 *      Author: Programming
 */

#include <EncoderDrivePID.h>
#include "SmartDashboard/SmartDashboard.h"
#include "LiveWindow/LiveWindow.h"

EncoderDrivePID::EncoderDrivePID(double p, double i, double d, DriveEncoders* driveEncoders,
		MecanumDriveTrain* driveTrain, int axis) : //axis: 2=straight, 1=straif
		PIDSubsystem("EncoderDrivePID", p, i, d) {
	this->p = p;
	this->i = i;
	this->d = d;
	this->driveEncoders = driveEncoders;
	this->driveTrain = driveTrain;
	this->axis = axis;
	SetSetpoint(0.0);
	Enable();

	// TODO Auto-generated constructor stub
}

void EncoderDrivePID::Enable() {
	GetPIDController()->Enable();
}
void EncoderDrivePID::SetSetpoint(double setpoint) {
	GetPIDController()->SetSetpoint(setpoint);
}

void EncoderDrivePID::Reset() {
	GetPIDController()->Reset();
}

void EncoderDrivePID::UsePIDOutput(double output) {
	switch (axis) {
		case STRAIGHT:
			driveTrain->SetStrafeOffset(output);
			break;
		case STRAFE:
			driveTrain->SetStraightOffset(-output);
			break;
		}
}

double EncoderDrivePID::ReturnPIDInput() {
	double input = 0;
	switch (axis) {
	case STRAFE:
		input = driveEncoders->GetDistanceStraight();
		break;
	case STRAIGHT:
		input = driveEncoders->GetDistanceStrafe();
		break;
	}
	return input;
}

void EncoderDrivePID::InitDefaultCommand() {

}

void EncoderDrivePID::Disable() {
	GetPIDController()->Disable();
}

EncoderDrivePID::~EncoderDrivePID() {
	// TODO Auto-generated destructor stub
}
