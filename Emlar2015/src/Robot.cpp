#include "WPILib.h"
#include "Joystick.h"
#include "OI.h"
#include "MechanumDriveTrain.h"
#include <stdio.h>
#include "GyroPID.h"
#include "AccelToDisp.h"

/**
 * This is a demo program showing how to use Mechanum control with the RobotDrive class.
 */
class Robot: public IterativeRobot
{
private:
    // Channels for the sensors, wheels, and joystick
	const uint32_t frontLeftChannel		= 2;
	const uint32_t backLeftChannel 		= 1;
	const uint32_t frontRightChannel 	= 0;
	const uint32_t backRightChannel 	= 3;
	const uint32_t gyroChannel			= 0;
	const uint32_t joystickChannel		= 0;

	//PID Coefficients
	double p = 0;
	double i = 0;
	double d = 0;

	//Sensors+outputs
	Talon* frontLeftWheel;
	Talon* backLeftWheel;
	Talon* frontRightWheel;
	Talon* backRightWheel;
	Accelerometer* accelerometer;
	Gyro* roboGyro;
	Joystick* stick;			//currently the only joystick

	//Controlling Objects
	AccelToDisp* accelToDisp;
	OI* opIn;
	MechanumDriveTrain* driveTrain;
	GyroPID* gyroPID;

	/*
	~Robot(){
		delete driveTrain;
		delete opIn;
	}
	*/


public:

	void RobotInit()
	{
		p=SmartDashboard::GetNumber("p");
		i = SmartDashboard::GetNumber("i");
		d = SmartDashboard::GetNumber("d");

		stick = new Joystick(joystickChannel);
		opIn = new OI(stick);
		frontLeftWheel = new Talon(frontLeftChannel);
		backLeftWheel = new Talon(backLeftChannel);
		frontRightWheel = new Talon(frontRightChannel);
		backRightWheel = new Talon(backRightChannel);
		accelerometer = new BuiltInAccelerometer();
		roboGyro = new Gyro(gyroChannel);
		driveTrain = new MechanumDriveTrain(frontLeftWheel, backLeftWheel,frontRightWheel,backRightWheel,opIn,roboGyro);
		gyroPID = new GyroPID(p,i,d,roboGyro,driveTrain);
		accelToDisp = new AccelToDisp(accelerometer);

		roboGyro->SetSensitivity(0.007);
		roboGyro->SetDeadband(0.0014);
		roboGyro->Reset();
	}

	~Robot() {
		delete gyroPID;
		delete accelToDisp;
		delete driveTrain;
		delete opIn;
		delete stick;
		delete roboGyro;
		delete accelerometer;
		delete frontLeftWheel;
		delete backLeftWheel;
		delete frontRightWheel;
		delete backRightWheel;
	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {
		gyroPID->SetSetpoint(0);
		SmartDashboard::PutNumber("JoystickY",opIn->GetX());
		SmartDashboard::PutNumber("JoystickX",opIn->GetY());
		SmartDashboard::PutNumber("JoystickTwist",opIn->GetTwist());
		SmartDashboard::PutNumber("GyroAngle",roboGyro->GetAngle());
		SmartDashboard::PutNumber("GyroAngle",roboGyro->GetRate());
		SmartDashboard::PutNumber("JoystickThrottle",opIn->GetThrottle());
	}

	void AutonomousInit() {
	}

	void AutonomousPeriodic() {
	}

};

START_ROBOT_CLASS(Robot);
