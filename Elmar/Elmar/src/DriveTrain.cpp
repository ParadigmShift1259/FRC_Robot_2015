/*
 * DriveTrain.cpp
 *
 *  Created on: Jan 16, 2015
 *      Author: Programming
 */
#include "WPIlib.h"

class DriveTrain
{
	Talon frontLeftChannel;
	Talon rearLeftChannel;
	Talon frontRightChannel;
	Talon rearRightChannel;
	RobotDrive robotDrive;	// robot drive system

	DriveTrain() :,
		frontLeftChannel(2),
		rearLeftChannel(1),
		frontRightChannel(0),
		rearRightChannel(3),
		robotDrive(frontLeftChannel, rearLeftChannel,
					   frontRightChannel, rearRightChannel)	// these must be initialized in the same order



	Drive() {
	}
private:

}


