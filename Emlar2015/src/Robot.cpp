#include "WPILib.h"
#include "OI.h"
#include "MechanumDriveTrain.h"
#include <stdio.h>
#include "GyroPID.h"
#include "AccelToDisp.h"
#include "Thermister.h"
#include "CorrectedGyro.h"

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
	const uint32_t gyroThermChannel		= 1;
	const uint32_t joystickChannel		= 0;

	//PID Coefficients
	double p = 0.04;
	double i = 0.00045;
	double d = 0.1;

	//Sensors+outputs
	Talon* frontLeftWheel;
	Talon* backLeftWheel;
	Talon* frontRightWheel;
	Talon* backRightWheel;
	Accelerometer* accelerometer;
	AnalogInput* gyro;
	AnalogInput* gyroThermister;
	CorrectedGyro* roboGyro;

	Joystick* stick;			//currently the only joystick

	//Controlling Objects
	AccelToDisp* accelToDisp;
	OI* opIn;
	MechanumDriveTrain* driveTrain;
	GyroPID* gyroPID;

	int count;
	double gyroSum;

public:

	void RobotInit()
	{
		/*
		p=SmartDashboard::GetNumber("p");
		i = SmartDashboard::GetNumber("i");
		d = SmartDashboard::GetNumber("d");
		*/
		stick = new Joystick(joystickChannel);
		opIn = new OI(stick);
		frontLeftWheel = new Talon(frontLeftChannel);
		backLeftWheel = new Talon(backLeftChannel);
		frontRightWheel = new Talon(frontRightChannel);
		backRightWheel = new Talon(backRightChannel);
		accelerometer = new BuiltInAccelerometer();
		gyro = new AnalogInput(gyroChannel);
		gyroThermister = new AnalogInput(gyroThermChannel);
		roboGyro = new CorrectedGyro(gyro,gyroThermister,2.5,0.3,0.7);
		driveTrain = new MechanumDriveTrain(frontLeftWheel, backLeftWheel,frontRightWheel,backRightWheel,opIn);
		gyroPID = new GyroPID(p,i,d,roboGyro,driveTrain);
		accelToDisp = new AccelToDisp(accelerometer);
	}

	~Robot() {
		//gyro stuff
		delete gyroPID;
		delete roboGyro;
		delete gyro;
		delete gyroThermister;

		delete accelToDisp;
		delete driveTrain;
		delete opIn;
		delete stick;
		delete accelerometer;

		//motors
		delete frontLeftWheel;
		delete backLeftWheel;
		delete frontRightWheel;
		delete backRightWheel;
	}

	void TeleopInit() {
		gyroPID->Enable();
		gyroPID->SetSetpoint(0.0);
		roboGyro->Reset();

	}

	void TeleopPeriodic() {
		driveTrain->Drive();
		SmartDashboard::PutNumber("JoystickY",opIn->GetX());
		SmartDashboard::PutNumber("JoystickX",opIn->GetY());
		SmartDashboard::PutNumber("JoystickTwist",opIn->GetTwist());
		SmartDashboard::PutNumber("GyroAngle",roboGyro->GetAngle());
		SmartDashboard::PutNumber("GyroAngle",roboGyro->GetRate());
		SmartDashboard::PutNumber("JoystickThrottle",opIn->GetThrottle());
	}

	void AutonomousInit() {
		count = 0;
		gyroSum = 0;
		gyroPID->Disable();
		printf("Temp, Voltage");
	}

	void AutonomousPeriodic() {
		count++;
		gyroSum = gyroSum+(roboGyro->GetVoltage());
		printf("%f, ",roboGyro->GetTemp());
		printf("%f\n",roboGyro->GetVoltage());

	}

	void DisabledInit() {
		//gyroPID->Disable();
	}

};

START_ROBOT_CLASS(Robot);
