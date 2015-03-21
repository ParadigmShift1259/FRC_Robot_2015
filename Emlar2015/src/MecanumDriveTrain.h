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
	const double pi =
			3.141592653589793238462643383279502884197169399375105820974944592307816406286;
	bool gyroPIDDisabled = false;

	double gyroPIDOffset = 0;
	double straightPIDOffset = 0;
	double strafePIDOffset = 0;
	double deadband = 1.0;
	double lastSpeed = 0;


	double drivecpr = 360.0 * 4.0;
	double driveGearRatio = 12.75;
	double driveWheelDiameter = 6.0;
	double driveRotationsPerInch = pi * driveWheelDiameter * driveGearRatio;
	double driveInchesPerClick = driveRotationsPerInch / drivecpr;

	double THRESHOLD = 1.0/driveInchesPerClick;
	double AUTODISTANCE = (7.0 * 12.0 + 6.0) / driveInchesPerClick;

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
			SpeedController* frontRight, SpeedController* backRight, OI* oI, DriveEncoders* driveEncoders) {
		operatorInputs = oI;
		frontLeftWheel = frontLeft;
		rearLeftWheel = backLeft;
		frontRightWheel = frontRight;
		rearRightWheel = backRight;
		this->driveEncoders = driveEncoders;
		robotDrive = new RobotDrive(frontRight,backLeft,backRight,frontLeft);
		robotDrive->SetExpiration(0.1);
		robotDrive->SetInvertedMotor(RobotDrive::kFrontRightMotor, false);// invert the left side motors
		robotDrive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, false);
		robotDrive->SetInvertedMotor(RobotDrive::kRearRightMotor, false);// you may need to change or remove this to match your robot
		robotDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor, false);
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
	bool DriveBackward(double speed);
	bool DriveLeft(double speed);
	void SetStraightOffset(double offset);
	void SetStrafeOffset(double offset);
	void Stop();
	double GetStraightDistance() {return (((double) driveEncoders->GetDistanceStraight())
			* driveInchesPerClick);}
	double GetStrafeDistance() {return (((double) driveEncoders->GetDistanceStrafe())
			* driveInchesPerClick);}
	double GetAUTODISTANCE() {return AUTODISTANCE;}
};

#endif /* SRC_MECHANUMDRIVETRAIN_H_ */
