/*
 * DriveTrain.cpp
 *
 *  Created on: Jan 16, 2015
 *      Author: Programming
 */
#include "WPIlib.h"

class MechanumDriveTrain
{
	Talon frontLeftWheel;
	Talon rearLeftWheel;
	Talon frontRightWheel;
	Talon rearRightWheel;
	RobotDrive robotDrive;	// robot drive system
	OI operatorInputs;

	MechanumDriveTrain(uint32_t frontLeftChannel,
			uint32_t backLeftChannel,
			uint32_t frontRightChannel,
			uint32_t backRightChannel, OI oI) :
		frontLeftWheel(frontLeftChannel),
		rearLeftWheel(backLeftChannel),
		frontRightWheel(frontRightChannel),
		rearRightWheel(backRightChannel),
		robotDrive(frontLeftWheel, rearLeftWheel,
					   frontRightWheel, rearRightWheel)	// these must be initialized in the same order
		{
			operatorInputs = oI;
			robotDrive.SetExpiration(0.1);
			robotDrive.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);	// invert the left side motors
			robotDrive.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);	// you may need to change or remove this to match your robot
		}
public:
	void Drive()
	{
		robotDrive.SetSafetyEnabled(false);
		robotDrive.MecanumDrive_Cartesian(OI.GetX(), OI.GetY(), OI.getRotation());
	}
private:

}


