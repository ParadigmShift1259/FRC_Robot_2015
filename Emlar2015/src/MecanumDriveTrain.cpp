/*
 * DriveTrain.cpp
 *
 *  Created on: Jan 16, 2015
 *      Author: Programming
 */
#include <MecanumDriveTrain.h>
#include <iostream>

	void MechanumDriveTrain::Drive() {
		double y = operatorInputs->GetY();
		double x = operatorInputs->GetX();
		double twist = -operatorInputs->GetTwist();
		robotDrive->MecanumDrive_Cartesian(
				y,
				x,
				//gyroPIDOffset
				twist);
	}

	void MechanumDriveTrain::DriveForward(double speed) {
		robotDrive->MecanumDrive_Cartesian(
				speed,
				xAccelPIDOffset,
				gyroPIDOffset);
	}

	void MechanumDriveTrain::DriveRight(double speed) {
		robotDrive->MecanumDrive_Cartesian(
				yAccelPIDOffset,
				speed,
				gyroPIDOffset);
	}

	void MechanumDriveTrain::Stop() {
		robotDrive->MecanumDrive_Cartesian(0,0,0);
	}

	void MechanumDriveTrain::SetXAccelOffset(double offset){
		xAccelPIDOffset = offset;
	}

	void MechanumDriveTrain::SetYAccelOffset(double offset){
		yAccelPIDOffset = offset;
	}

	void MechanumDriveTrain::SetZAccelOffset(double offset){
		zAccelPIDOffset = offset;
	}

	void MechanumDriveTrain::SetGyroOffset(double offset) {
		gyroPIDOffset = offset;
	}
