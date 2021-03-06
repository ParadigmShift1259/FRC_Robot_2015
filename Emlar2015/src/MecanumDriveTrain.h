/*
 * MecanumDriveTrain.h
 *
 *  Created on: Jan 17, 2015
 *      Author: Programming
 */
#include "OI.h"
#include "DriveEncoders.h"

#ifndef SRC_MECANUMDRIVETRAIN_H_
#define SRC_MECANUMDRIVETRAIN_H_

class MecanumDriveTrain {
private:
	bool gyroPIDDisabled;

	double gyroPIDOffset = 0;
	double straightPIDOffset = 0;
	double strafePIDOffset = 0;
	double deadband = 1.0;
	double lastSpeed = 0;

	SpeedController * frontLeftWheel;
	SpeedController* rearLeftWheel;
	SpeedController* frontRightWheel;
	SpeedController* rearRightWheel;

	OI* operatorInputs;
	RobotDrive* robotDrive;	// robot drive system
	DriveEncoders* driveEncoders;

	/*
	 ~MechanumDriveTrain() {
	 delete operatorInputs;
	 }
	 */

public:

	MecanumDriveTrain(SpeedController* frontLeft, SpeedController* backLeft,
			SpeedController* frontRight, SpeedController* backRight, OI* oI) {
		operatorInputs = oI;
		frontLeftWheel = frontLeft;
		rearLeftWheel = backLeft;
		frontRightWheel = frontRight;
		rearRightWheel = backRight;
		robotDrive = new RobotDrive(frontLeft, frontRight, backLeft, backRight);
		robotDrive->SetExpiration(0.1);
		robotDrive->SetInvertedMotor(RobotDrive::kFrontRightMotor, true);// invert the left side motors
		robotDrive->SetInvertedMotor(RobotDrive::kRearRightMotor, true);// you may need to change or remove this to match your robot
		robotDrive->SetSafetyEnabled(false);
	}
	void Drive();
	void Turn(double degrees, double gyroAngle);
	bool GyroPIDDisabled();
	void EnableTurn();
	void SetGyroOffset(double offset);
	void SetAccelOffset(double offset);
	bool DriveForward(double speed);
	bool DriveRight(double speed);
	void SetStraightOffset(double offset);
	void SetStrafeOffset(double offset);
	void SetDriveEncoders(DriveEncoders* driveEncoders);
	void Stop();

};

#endif /* SRC_MECHANUMDRIVETRAIN_H_ */
