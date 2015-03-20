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
	double x = -operatorInputs->GetX();
	double twist = operatorInputs->GetTwist();
	if (operatorInputs->GetToggleTrigger()) {
		robotDrive->MecanumDrive_Cartesian(y, x, gyroPIDOffset);
	} else {
		robotDrive->MecanumDrive_Cartesian(y, x, twist);
	}
}

bool MecanumDriveTrain::DriveForward(double distance) {
	double distanceDriven = driveEncoders->GetDistanceStraight();
	double speed = 0;
	if (distanceDriven > distance) {
		speed = 0;
		lastSpeed = 0;
	} else if (distanceDriven > (.5 * (distance))) {
		speed = lastSpeed + .001;
	} else {
		if (lastSpeed != .001 && distanceDriven != distance) {
			speed = lastSpeed - .01;
		}
	}
	lastSpeed = speed;
	robotDrive->MecanumDrive_Cartesian(strafePIDOffset, -speed, gyroPIDOffset);
	return (distanceDriven == distance);
}

bool MecanumDriveTrain::DriveRight(double distance) {
	double distanceDriven = driveEncoders->GetDistanceStraight();
	double speed = 0;
	if ((distanceDriven - distance) < THRESHOLD) {
		return true;
	}
	if (distanceDriven < (.5 * (distance))) {
		speed = lastSpeed + .01;
	} else if (lastSpeed != .01 && distanceDriven < distance) {
		speed = lastSpeed - .01;
	} else {
		speed = -.05;
	}
	lastSpeed = speed;
	robotDrive->MecanumDrive_Cartesian(straightPIDOffset, -speed, gyroPIDOffset);
	return false;
}
bool MecanumDriveTrain::DriveBackward(double distance) {
	double distanceDriven = -driveEncoders->GetDistanceStraight();
	double speed = 0;
	if ((distanceDriven - distance) < THRESHOLD) {
		return true;
	}
	if (distanceDriven < (.5 * (distance))) {
		speed = lastSpeed + .01;
	} else if (lastSpeed != .01 && distanceDriven < distance) {
		speed = lastSpeed - .01;
	} else {
		speed = -.05;
	}
	lastSpeed = speed;
	robotDrive->MecanumDrive_Cartesian(straightPIDOffset, speed, gyroPIDOffset);
	return false;
}

bool MecanumDriveTrain::DriveLeft(double distance) {
	double distanceDriven = -driveEncoders->GetDistanceStrafe();
	double speed = 0;
	if ((distanceDriven - distance) < THRESHOLD) {
		return true;
	}
	if (distanceDriven < (.5 * (distance))) {
		speed = lastSpeed + .01;
	} else if (lastSpeed != .01 && distanceDriven < distance) {
		speed = lastSpeed - .01;
	} else {
		speed = -.05;
	}
	lastSpeed = speed;
	robotDrive->MecanumDrive_Cartesian(speed, straightPIDOffset, gyroPIDOffset);
	return false;
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
	double y = operatorInputs->GetY();
	double x = operatorInputs->GetX();
	if (error > deadband || error < -deadband) {
		if (error > 0) {
			robotDrive->MecanumDrive_Cartesian(y, x, -1.0);
		}
		if (error < 0) {
			robotDrive->MecanumDrive_Cartesian(y, x, 1.0);
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
	gyroPIDOffset = -offset;
}
