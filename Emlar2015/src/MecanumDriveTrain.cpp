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
		double x = straifPIDOffset;
		double twist = gyroPIDOffset;
		robotDrive->MecanumDrive_Cartesian(
				y,
				x,
				//gyroPIDOffset
				twist);
	}

	void MechanumDriveTrain::DriveForward(double distance) {
		robotDrive->MecanumDrive_Cartesian(
				distance,
				straifPIDOffset,
				gyroPIDOffset);
	}

	void MechanumDriveTrain::DriveRight(double speed) {
		robotDrive->MecanumDrive_Cartesian(
				straightPIDOffset,
				speed,
				gyroPIDOffset);
	}

	void MechanumDriveTrain::Stop() {
		robotDrive->MecanumDrive_Cartesian(0,0,0);
	}

	void MechanumDriveTrain::SetStraifOffset(double offset){
		straifPIDOffset = offset;
	}

	void MechanumDriveTrain::SetStraightOffset(double offset){
		straightPIDOffset = offset;
	}

	void MechanumDriveTrain::SetGyroOffset(double offset) {
		gyroPIDOffset = offset;
	}
