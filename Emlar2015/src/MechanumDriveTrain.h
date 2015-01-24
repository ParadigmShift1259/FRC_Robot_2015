/*
 * MechanumDriveTrain.h
 *
 *  Created on: Jan 17, 2015
 *      Author: Programming
 */
#include "OI.h"
#ifndef SRC_MECHANUMDRIVETRAIN_H_
#define SRC_MECHANUMDRIVETRAIN_H_

class MechanumDriveTrain {
private:
	SpeedController* frontLeftWheel;
	SpeedController* rearLeftWheel;
	SpeedController* frontRightWheel;
	SpeedController* rearRightWheel;
	OI* operatorInputs;
	RobotDrive* robotDrive;	// robot drive system

	/*
	~MechanumDriveTrain() {
		delete operatorInputs;
	}
	*/


public:

	double gyroPIDOffset=0;

	MechanumDriveTrain(
			SpeedController* frontLeft,
			SpeedController* backLeft,
			SpeedController* frontRight,
			SpeedController* backRight,
			OI* oI)
	{
			operatorInputs = oI;
			frontLeftWheel = frontLeft;
			rearLeftWheel = backLeft;
			frontRightWheel = frontRight;
			rearRightWheel = backRight;
			robotDrive = new RobotDrive(frontLeft,frontRight,backLeft,backRight);
			robotDrive->SetExpiration(0.1);
			robotDrive->SetInvertedMotor(RobotDrive::kFrontRightMotor, true);	// invert the left side motors
			robotDrive->SetInvertedMotor(RobotDrive::kRearRightMotor, true);// you may need to change or remove this to match your robot
			robotDrive->SetSafetyEnabled(false);
		}
	void Drive();

	void SetGyroPIDOffset(double offset);

};



#endif /* SRC_MECHANUMDRIVETRAIN_H_ */
