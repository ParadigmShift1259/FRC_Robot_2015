/*
 * DriveEncoder.cpp
 *
 *  Created on: Jan 31, 2015
 *      Author: paradigm
 */

#include <DriveEncoders.h>

/**
 * constructor that takes in 4 encoders for the mecanum drive train
 */
DriveEncoders::DriveEncoders(CANTalon* frontRightTalon,
		CANTalon* backRightTalon, CANTalon* frontLeftTalon,
		CANTalon* backLeftTalon) {
	this->frontLeftTalon = frontLeftTalon;
	this->backLeftTalon = frontLeftTalon;
	this->frontRightTalon = frontRightTalon;
	this->backRightTalon = backRightTalon;
}

/**
 * gets the distance traveled forward from the encoders in inches
 */
double DriveEncoders::GetDistanceStraight() {
	double forwardDistance = (GetFrontRight()
			+ GetBackLeft() - GetFrontRight()
			- GetBackRight()) / 4.0;
	return forwardDistance;
}

/**
 * gets the distance traveled right from the encoders in inches
 */

double DriveEncoders::GetDistanceStrafe() {
	double sidewaysDistance = (GetFrontLeft()
			+ GetBackLeft() + GetFrontRight()
			+ GetBackRight()) / 4.0;
	return sidewaysDistance;
}

double DriveEncoders::GetRotation() {
	double rotation = (GetFrontLeft()
			+ GetBackLeft() - GetFrontRight()
			- GetBackRight()) / 4.0;
	return rotation;
}

double DriveEncoders::GetFrontLeft() {
	return frontLeftTalon->GetPosition();
}
double DriveEncoders::GetFrontRight() {
	return -frontRightTalon->GetPosition();
}
double DriveEncoders::GetBackLeft() {
	return backLeftTalon->GetPosition();
}
double DriveEncoders::GetBackRight() {
	return backRightTalon->GetPosition();
}




/**
 * resets the encoders to zero
 */
void DriveEncoders::ResetEncoders() {
	frontLeftTalon->SetPosition(0);
	frontRightTalon->SetPosition(0);
	backLeftTalon->SetPosition(0);
	backRightTalon->SetPosition(0);
}

DriveEncoders::~DriveEncoders() {
	// TODO Auto-generated destructor stub
}
