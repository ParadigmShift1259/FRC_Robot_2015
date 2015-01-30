/*
 * DriveTrain.cpp
 *
 *  Created on: Jan 16, 2015
 *      Author: Programming
 */
#include "MechanumDriveTrain.h"
#include <iostream>

	void MechanumDriveTrain::Drive() {
		double y = operatorInputs->GetY();
		double twist = -operatorInputs->GetTwist();
		printf("Got OI Values\n");

		printf("Just Before Driving\n");
		robotDrive->MecanumDrive_Cartesian(
				y,
				accelPIDOffset,
				//gyroPIDOffset
				twist);
		printf("Driving\n");
	}

	void MechanumDriveTrain::SetAccelOffset(double offset){
		accelPIDOffset = offset;
		printf("SettingAccelOffset\n");
	}

	void MechanumDriveTrain::SetGyroOffset(double offset) {
		gyroPIDOffset = offset;
		SmartDashboard::PutNumber("GyroPIDOutput",offset);
	}
