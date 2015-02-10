/*
 * DriveTrain.cpp
 *
 *  Created on: Jan 16, 2015
 *      Author: Programming
 */
#include <MecanumDriveTrain.h>
#include <iostream>

void MecanumDriveTrain::Drive() {
	double y = operatorInputs->GetY();
	double x = operatorInputs->GetX();
	double twist = operatorInputs->GetTwist();
	if (operatorInputs->GetTrigger()) {
		robotDrive->MecanumDrive_Cartesian(y, x, gyroPIDOffset);
	} else {
		robotDrive->MecanumDrive_Cartesian(y, x, twist);
	}
}

void MecanumDriveTrain::DriveForward(double speed) {
	robotDrive->MecanumDrive_Cartesian(speed, strafePIDOffset,
			gyroPIDOffset);
}

void MecanumDriveTrain::DriveRight(double speed) {
	robotDrive->MecanumDrive_Cartesian(straightPIDOffset, speed,
			gyroPIDOffset);
}

void MecanumDriveTrain::Stop() {
	robotDrive->MecanumDrive_Cartesian(0, 0, 0);
}

void MecanumDriveTrain::SetStrafeOffset(double offset) {
	strafePIDOffset = offset;
}

bool MecanumDriveTrain::GyroPIDDisabled() {
	return gyroPIDDisabled;
}

void MecanumDriveTrain::EnableTurn() {
	gyroPIDDisabled = true;
}

void MecanumDriveTrain::Turn(double degrees, double gyroAngle) {
	double error = degrees - gyroAngle;
	if (error > deadband || error < -deadband) {
		if (error > 0) {
			robotDrive->MecanumDrive_Cartesian(0, 0, 1.0);
		}
		if (error < 0) {
			robotDrive->MecanumDrive_Cartesian(0, 0, -1.0);
		}
		SmartDashboard::GetNumber("Error", error);
	} else {
		gyroPIDDisabled = false;
	}
}

void MecanumDriveTrain::SetStraightOffset(double offset) {
	straightPIDOffset = offset;
}

void MecanumDriveTrain::SetGyroOffset(double offset) {
	gyroPIDOffset = offset;
}
