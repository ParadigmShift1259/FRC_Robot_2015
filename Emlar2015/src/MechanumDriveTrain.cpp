/*
 * DriveTrain.cpp
 *
 *  Created on: Jan 16, 2015
 *      Author: Programming
 */
#include "MechanumDriveTrain.h"
#include <iostream>

	void MechanumDriveTrain::Drive() {

		robotDrive->MecanumDrive_Cartesian(
				operatorInputs->GetY(),
				operatorInputs->GetX(),
				-operatorInputs->GetTwist());
	}

	void MechanumDriveTrain::SetGyroPIDOffset(double offset) {
		gyroPIDOffset = offset;
		SmartDashboard::PutNumber("GyroPIDOutput",offset);
	}
