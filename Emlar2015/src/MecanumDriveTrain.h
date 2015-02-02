/*
 * MechanumDriveTrain.h
 *
 *  Created on: Jan 17, 2015
 *      Author: Programming
 */
#include "OI.h"
#include "DriveEncoders.h"

#ifndef SRC_MECHANUMDRIVETRAIN_H_
#define SRC_MECHANUMDRIVETRAIN_H_

class MechanumDriveTrain {
private:
	double gyroPIDOffset		= 0;
	double straightPIDOffset	= 0;
	double straifPIDOffset		= 0;

	SpeedController* frontLeftWheel;
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

	MechanumDriveTrain(SpeedController* frontLeft, SpeedController* backLeft,
			SpeedController* frontRight, SpeedController* backRight,
			OI* oI,DriveEncoders* driveEncoders) {
		operatorInputs = oI;
		frontLeftWheel = frontLeft;
		rearLeftWheel = backLeft;
		frontRightWheel = frontRight;
		rearRightWheel = backRight;
		this->driveEncoders = driveEncoders;
		robotDrive = new RobotDrive(frontLeft, frontRight, backLeft, backRight);
		robotDrive->SetExpiration(0.1);
		robotDrive->SetInvertedMotor(RobotDrive::kFrontRightMotor, true);// invert the left side motors
		robotDrive->SetInvertedMotor(RobotDrive::kRearRightMotor, true);// you may need to change or remove this to match your robot
		robotDrive->SetSafetyEnabled(false);
	}
	void Drive();

	void SetGyroOffset(double offset);
	void SetAccelOffset(double offset);
	void DriveForward(double speed);
	void DriveRight(double speed);
	void SetStraightOffset(double offset);
	void SetStraifOffset(double offset);
	void Stop();

};

#endif /* SRC_MECHANUMDRIVETRAIN_H_ */
