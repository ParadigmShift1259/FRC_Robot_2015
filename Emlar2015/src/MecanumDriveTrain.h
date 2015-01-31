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
	double gyroPIDOffset=0;
	double xAccelPIDOffset=0;
	double yAccelPIDOffset=0;
	double zAccelPIDOffset=0;

	SpeedController* frontLeftWheel;
	SpeedController* rearLeftWheel;
	SpeedController* frontRightWheel;
	SpeedController* rearRightWheel;

	Encoder* frontLeftEncoder;
	Encoder* rearLeftEncoder;
	Encoder* frontRightEncoder;
	Encoder* rearRightEncoder;
	OI* operatorInputs;
	RobotDrive* robotDrive;	// robot drive system

	/*
	~MechanumDriveTrain() {
		delete operatorInputs;
	}
	*/


public:


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

	void SetGyroOffset(double offset);
	void SetAccelOffset(double offset);
	void DriveForward(double speed);
	void DriveRight(double speed);
	void SetXAccelOffset(double offset);
	void SetYAccelOffset(double offset);
	void SetZAccelOffset(double offset);
	void Stop();

};



#endif /* SRC_MECHANUMDRIVETRAIN_H_ */
