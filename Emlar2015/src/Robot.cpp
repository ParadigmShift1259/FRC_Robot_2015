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
	double gyroP = 0.04;
	double gyroI = 0.00045;
	double gyroD = 0.1;

	//Outputs
	Talon* frontLeftWheel;
	Talon* backLeftWheel;
	Talon* frontRightWheel;
	Talon* backRightWheel;

	//Sensors
	Accelerometer* accelerometer;
	AnalogInput* gyro;
	AnalogInput* gyroThermister;
	Joystick* stick;			//currently the only joystick

	//Controlling Objects
	AccelToDisp* accelToDisp;
	OI* opIn;
	MechanumDriveTrain* driveTrain;
	CorrectedGyro* roboGyro;
	GyroPID* gyroPID;

	int count;
	double gyroSum;

public:

	void RobotInit()
	{
		stick = new Joystick(joystickChannel);
		opIn = new OI(stick);
		frontLeftWheel = new Talon(frontLeftChannel);
		backLeftWheel = new Talon(backLeftChannel);
		frontRightWheel = new Talon(frontRightChannel);
		backRightWheel = new Talon(backRightChannel);
		accelerometer = new BuiltInAccelerometer();
		gyro = new AnalogInput(gyroChannel);
		gyroThermister = new AnalogInput(gyroThermChannel);
		roboGyro = new CorrectedGyro(gyro,gyroThermister);
		driveTrain = new MechanumDriveTrain(frontLeftWheel, backLeftWheel,frontRightWheel,backRightWheel,opIn);
		gyroPID = new GyroPID(gyroP,gyroI,gyroD,roboGyro,driveTrain);
		accelToDisp = new AccelToDisp(accelerometer);
		double avgValue = gyro->GetValue()+gyro->GetValue()+gyro->GetValue()+gyro->GetValue()+gyro->GetValue();
		avgValue = avgValue/5;
		roboGyro->SetZeroVoltage(avgValue);
		roboGyro->SetDeadband(6);
		roboGyro->SetSensitivity(0.007);
		roboGyro->Start();
		roboGyro->Reset();
	}

	~Robot() {
		//motors
		delete frontLeftWheel;
		delete backLeftWheel;
		delete frontRightWheel;
		delete backRightWheel;

		//gyro stuff
		delete gyroPID;
		delete roboGyro;
		delete gyro;
		delete gyroThermister;

		//controlling objects
		delete accelToDisp;
		delete driveTrain;
		delete opIn;
		delete stick;
		delete accelerometer;
	}

	void TeleopInit() {
		gyroPID->Enable();
		gyroPID->SetSetpoint(0.0);

	}

	void TeleopPeriodic() {
		driveTrain->Drive();
		SmartDashboard::PutNumber("JoystickY",opIn->GetX());
		SmartDashboard::PutNumber("JoystickX",opIn->GetY());
		SmartDashboard::PutNumber("JoystickTwist",opIn->GetTwist());
		SmartDashboard::PutNumber("GyroAngle",roboGyro->GetAngle());
		SmartDashboard::PutNumber("GyroAngle",roboGyro->GetRate());
		SmartDashboard::PutNumber("JoystickThrottle",opIn->GetThrottle());
		printf("%f,GyroAngle",roboGyro->GetAngle());
		printf("%f,GyroRate",roboGyro->GetRate());
	}

	void TestInit() {
		/*
		count = 0;
		gyroSum = 0;
		gyroPID->Disable();
		printf("Temp, Voltage");
		*/
		double avgValue = gyro->GetValue()+gyro->GetValue()+gyro->GetValue()+gyro->GetValue()+gyro->GetValue();
		avgValue = avgValue/5;
		roboGyro->SetZeroVoltage(avgValue);
		roboGyro->Reset();
	}

	void TestPeriodic() {
		count++;
		gyroSum = gyroSum+(roboGyro->GetVoltage());
		printf("%f, ",roboGyro->GetTemp());
		printf("%f, ",roboGyro->GetVoltage());
		printf("%f, ",roboGyro->GetRawValue());
		printf("%f\n",roboGyro->GetAngle());
	}

	void DisabledInit() {
		gyroPID->Reset();
		gyroPID->Disable();
	}

};

START_ROBOT_CLASS(Robot);
