/*
 * DriveEncoder.cpp
 *
 *  Created on: Jan 31, 2015
 *      Author: paradigm
 */

#include <DriveEncoders.h>

DriveEncoders::DriveEncoders(Encoder* frontLeftEncoder,
		Encoder* rearLeftEncoder, Encoder* frontRightEncoder,
		Encoder* rearRightEncoder) {
	this->frontLeftEncoder = frontLeftEncoder;
	this->rearLeftEncoder = frontLeftEncoder;
	this->frontRightEncoder = frontRightEncoder;
	this->rearRightEncoder = rearRightEncoder;
}

double DriveEncoders::GetDistanceStraight() {
	double forwardDistance = (frontLeftEncoder->GetDistance()
			+ rearLeftEncoder->GetDistance() + frontRightEncoder->GetDistance()
			+ rearRightEncoder->GetDistance()) / 4;
	return forwardDistance;
}

double DriveEncoders::GetDistanceStrafe() {
	double sidewaysDistance = (frontLeftEncoder->GetDistance()
			+ rearLeftEncoder->GetDistance() - frontRightEncoder->GetDistance()
			- rearRightEncoder->GetDistance()) / 4;
	return sidewaysDistance;
}

double DriveEncoders::GetRotation() {
	double rotation = (frontLeftEncoder->GetDistance()
			+ rearLeftEncoder->GetDistance() - frontRightEncoder->GetDistance()
			- rearRightEncoder->GetDistance()) / 4;
	return rotation;
}

void DriveEncoders::ResetEncoders() {
	frontLeftEncoder->Reset();
	frontRightEncoder->Reset();
	rearLeftEncoder->Reset();
	rearRightEncoder->Reset();
}

DriveEncoders::~DriveEncoders() {
	// TODO Auto-generated destructor stub
}

