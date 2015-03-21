/*
 * DriveTrain.cpp
 *
 *  Created on: Jan 16, 2015
 *      Author: Programming
 */
#include <MecanumDriveTrain.h>
#include <iostream>
#include <math.h>

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
	double distanceDriven = -driveEncoders->GetDistanceStraight();
	double speed = 0.0;
	if (std::abs(distance - distanceDriven) < THRESHOLD) {
		speed = 0.0;
	} else if (distanceDriven < (0.5 * (distance))) {
		speed = lastSpeed + 0.001;
	} else if (lastSpeed > 0.001) {
		speed = lastSpeed - 0.001;
	} else {
		speed = 0.001;
	}
	lastSpeed = speed;
	robotDrive->MecanumDrive_Cartesian(strafePIDOffset, -speed, gyroPIDOffset);
	SmartDashboard::PutNumber("DistanceForwards", distanceDriven);
	SmartDashboard::PutNumber("SpeedForwards",speed);
	return (std::abs(distanceDriven - distance) < THRESHOLD);
}
bool MecanumDriveTrain::DriveBackward(double distance) {
	double distanceDriven = driveEncoders->GetDistanceStraight();
	double speed = 0.0;
	if (std::abs(distance - distanceDriven) < THRESHOLD) {
		speed = 0.0;
	} else if (distanceDriven < (0.5 * (distance))) {
		speed = lastSpeed + 0.001;
	} else if (lastSpeed != 0.001) {
		speed = lastSpeed - 0.001;
	}
	lastSpeed = speed;
	robotDrive->MecanumDrive_Cartesian(strafePIDOffset, speed, gyroPIDOffset);
	SmartDashboard::PutNumber("DistanceBackwards", distanceDriven);
	SmartDashboard::PutNumber("SpeedBackwards",speed);
	return (std::abs(distanceDriven - distance) < THRESHOLD);
}

bool MecanumDriveTrain::DriveRight(double distance) {
	double distanceDriven = -driveEncoders->GetDistanceStrafe();
	double speed = 0.0;
	if (std::abs(distance - distanceDriven) < THRESHOLD) {
		speed = 0.0;
	} else if (distanceDriven < (0.5 * (distance))) {
		speed = lastSpeed + 0.001;
	} else if (lastSpeed != 0.001) {
		speed = lastSpeed - 0.001;
	}
	lastSpeed = speed;
	robotDrive->MecanumDrive_Cartesian(speed, straightPIDOffset, gyroPIDOffset);
	SmartDashboard::PutNumber("DistanceRight", distanceDriven);
	SmartDashboard::PutNumber("SpeedRight",speed);
	return (std::abs(distanceDriven - distance) < THRESHOLD);
}

bool MecanumDriveTrain::DriveLeft(double distance) {
	double distanceDriven = driveEncoders->GetDistanceStrafe();
	double speed = 0.0;
	if (std::abs(distance - distanceDriven) < THRESHOLD) {
		speed = 0.0;
	} else if (distanceDriven < (0.5 * (distance))) {
		speed = lastSpeed + 0.001;
	} else if (lastSpeed > 0.001) {
		speed = lastSpeed - 0.001;
	} else {
		speed = .001;
	}
	lastSpeed = speed;
	robotDrive->MecanumDrive_Cartesian(-speed, strafePIDOffset, gyroPIDOffset);
	SmartDashboard::PutNumber("DistanceLeft", distanceDriven);
	SmartDashboard::PutNumber("SpeedLeft",speed);
	return (std::abs(distanceDriven - distance) < THRESHOLD);}

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
