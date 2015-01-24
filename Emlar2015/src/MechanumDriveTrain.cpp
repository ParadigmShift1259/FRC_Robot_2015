/*
 * DriveTrain.cpp
 *
 *  Created on: Jan 16, 2015
 *      Author: Programming
 */
#include "MechanumDriveTrain.h"
#include <iostream>

	void MechanumDriveTrain::Drive() {
		if(operatorInputs->GetTrigger())
		{
			robotDrive->SetMaxOutput(0.5);
		} else {
			robotDrive->SetMaxOutput(1.0);
		}
		robotDrive->MecanumDrive_Cartesian(
				operatorInputs->GetY(),
				operatorInputs->GetX(),
				gyroPIDOffset);
	}

	void MechanumDriveTrain::SetGyroPIDOffset(double offset) {
		gyroPIDOffset = offset;
		SmartDashboard::PutNumber("GyroPIDOutput",offset);
	}
