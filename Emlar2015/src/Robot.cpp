#include "WPILib.h"
#include "Joystick.h"
#include "OI.h"
#include "MechanumDriveTrain.h"
#include <stdio.h>
#include "GyroPID.h"
#include "CorrectedGyro.h"
#include "AccelPID.h"

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
	const uint32_t compressorChannel    = 0;

	//PID Coefficients
	double gyroP = 0.04;
	double gyroI = 0.00045;
	double gyroD = 0.1;

	double accelStraifP = .1;
	double accelStraifI = .0;
	double accelStraifD = .0;


	//inputs
	Accelerometer* accelerometer;
	AnalogInput* therm;
	AnalogInput* gyro;
	Joystick* stick;			//currently the only joystick

	//outputs
	Talon* frontLeftWheel;
	Talon* backLeftWheel;
	Talon* frontRightWheel;
	Talon* backRightWheel;
	//Compressor* compressor;

	//Controlling Objects
	OI* opIn;
	MechanumDriveTrain* driveTrain;
	GyroPID* gyroPID;
	AccelPID* accelPID;
	CorrectedGyro* roboGyro;

public:

	void RobotInit()
	{
		//inputs
		therm = new AnalogInput(gyroThermChannel);
		gyro = new AnalogInput(gyroChannel);
		stick = new Joystick(joystickChannel);
		accelerometer = new BuiltInAccelerometer();

		//outputs
		//compressor = new Compressor(compressorChannel);
		frontLeftWheel = new Talon(frontLeftChannel);
		backLeftWheel = new Talon(backLeftChannel);
		frontRightWheel = new Talon(frontRightChannel);
		backRightWheel = new Talon(backRightChannel);

		//wpilib class setup
		//compressor->SetClosedLoopControl(true);

		//user generated classes
		opIn = new OI(stick);
		driveTrain = new MechanumDriveTrain(frontLeftWheel, backLeftWheel,frontRightWheel,backRightWheel,opIn);
		roboGyro = new CorrectedGyro(gyro,therm);
		gyroPID = new GyroPID(gyroP,gyroI,gyroD,roboGyro,driveTrain);
		accelPID = new AccelPID(accelStraifP,accelStraifI,accelStraifD,accelerometer,driveTrain);

		//user generated classes setup
		double averageValue = (roboGyro->GetRaw() + roboGyro->GetRaw() + roboGyro->GetRaw()+ roboGyro->GetRaw()+ roboGyro->GetRaw())/5;
		roboGyro->SetZeroValue(averageValue);
		roboGyro->SetSensitivity(0.007);
		roboGyro->SetDeadband(5);
		roboGyro->Reset();
	}

	~Robot() {
		//inputs
		delete therm;
		delete gyro;
		delete stick;
		delete accelerometer;

		//outputs
		delete frontLeftWheel;
		delete backLeftWheel;
		delete frontRightWheel;
		delete backRightWheel;
		//delete compressor;

		//user generated classes
		delete gyroPID;
		delete accelPID;
		delete roboGyro;
		delete driveTrain;
		delete opIn;
	}

	/*
	 * preps for the human operated mode
	 */
	void TeleopInit() {
		printf("TeleopInit \n");
		roboGyro->Reset(); //resets the Gyro
		//gyroPID->Enable(); //enables PID
		//gyroPID->SetSetpoint(0.0); //sets the setpoint to zero
		accelPID->Enable();
		accelPID->SetSetpoint(0.0);
		//compressor->Enabled(); //enables compressor

	}

	/*
	 * human operated mode loop
	 */
	void TeleopPeriodic() {
		printf("InTeleop\n");
		driveTrain->Drive(); //tells the robot to drive
		//compressor->Enabled(); //enables the compressor

		//put numbers to the smart dashboard for diagnostics
		SmartDashboard::PutNumber("JoystickY",opIn->GetX());
		SmartDashboard::PutNumber("JoystickX",opIn->GetY());
		SmartDashboard::PutNumber("JoystickTwist",opIn->GetTwist());
		SmartDashboard::PutNumber("GyroAngle",roboGyro->GetAngle());
		SmartDashboard::PutNumber("GyroAngle",roboGyro->GetRate());
		SmartDashboard::PutNumber("JoystickThrottle",opIn->GetThrottle());
	}

	/*
	 * preps for autonomous mode
	 */
	void AutonomousInit() {
		roboGyro->Reset();
		//compressor->Enabled();
	}

	/*
	 * autonomous mode loop
	 */
	void AutonomousPeriodic() {
		printf("Angle= %f\n",roboGyro->GetAngle());
		printf("Raw= %f\n",roboGyro->GetRaw());
		//compressor->Enabled();
		while(IsAutonomous()) {
		}
	}

	/*
	 * preps for switch to disabled
	 */
	void DisabledInit() {
		//compressor->SetClosedLoopControl(false);
		accelPID->Reset();
		accelPID->Disable();
		gyroPID->Disable(); //stops the gyro PID
	}

};

START_ROBOT_CLASS(Robot);
