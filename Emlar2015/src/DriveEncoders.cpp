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
DriveEncoders::DriveEncoders(Encoder* frontLeftEncoder,
		Encoder* rearLeftEncoder, Encoder* frontRightEncoder,
		Encoder* rearRightEncoder) {
	this->frontLeftEncoder = frontLeftEncoder;
	this->rearLeftEncoder = frontLeftEncoder;
	this->frontRightEncoder = frontRightEncoder;
	this->rearRightEncoder = rearRightEncoder;
}

/**
 * gets the distance traveled forward from the encoders in inches
 */
double DriveEncoders::GetDistanceStraight() {
	double forwardDistance = (frontLeftEncoder->GetDistance()
					+ rearLeftEncoder->GetDistance() - frontRightEncoder->GetDistance()
					- rearRightEncoder->GetDistance()) / 4;
	return forwardDistance;
}

/**
 * gets the distance traveled right from the encoders in inches
 */

double DriveEncoders::GetDistanceStrafe() {
	double sidewaysDistance = (frontLeftEncoder->GetDistance()
			+ rearLeftEncoder->GetDistance() + frontRightEncoder->GetDistance()
			+ rearRightEncoder->GetDistance()) / 4;
	return sidewaysDistance;
}

double DriveEncoders::GetRotation() {
	double rotation = (frontLeftEncoder->GetDistance()
			+ rearLeftEncoder->GetDistance() - frontRightEncoder->GetDistance()
			- rearRightEncoder->GetDistance()) / 4;
	return rotation;
}

/**
 * resets the encoders to zero
 */
void DriveEncoders::ResetEncoders() {
	frontLeftEncoder->Reset();
	frontRightEncoder->Reset();
	rearLeftEncoder->Reset();
	rearRightEncoder->Reset();
}

DriveEncoders::~DriveEncoders() {
	// TODO Auto-generated destructor stub
}
