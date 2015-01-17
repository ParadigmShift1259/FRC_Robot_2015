/*
 * DriveTrain.cpp
 *
 *  Created on: Jan 16, 2015
 *      Author: Programming
 */
#include "MechanumDriveTrain.h"
#include <iostream>

	void MechanumDriveTrain::Drive() {
		operatorInputs.deadD();
		std::cout << operatorInputs.getX()<<std::endl;
		robotDrive.MecanumDrive_Cartesian(operatorInputs.getX(),operatorInputs.getY(),
				operatorInputs.getTwist());
	}

