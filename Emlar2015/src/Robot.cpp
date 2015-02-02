#include <EncoderDrivePID.h>
#include <MecanumDriveTrain.h>
#include "WPILib.h"
#include "Joystick.h"
#include "OI.h"
#include <stdio.h>
#include "GyroPID.h"
#include "CorrectedGyro.h"
#include "Vacuum.h"
#include "DriveEncoders.h"
#include "EncoderDrivePID.h"

/**
 * This is a demo program showing how to use Mechanum control with the RobotDrive class.
 */
class Robot: public IterativeRobot
{

private:
	//Strafe and Straight Definitions
	static const int STRAIGHT = 1;
	static const int STRAFE = 2;

	//PDP Channels
	const uint32_t frontLeftPDPChannel			= 0;
	const uint32_t frontRightPDPChannel			= 2;
	const uint32_t backLeftPDPChannel			= 1;
	const uint32_t backRightPDPChannel			= 3;

    // Channels for the motors
	const uint32_t frontLeftChannel				= 2;
	const uint32_t frontRightChannel 			= 0;
	const uint32_t backLeftChannel 				= 1;
	const uint32_t backRightChannel 			= 3;
	const uint32_t vacuumMotor1Channel			= 4;

	//Drive Encoders
	const uint32_t frontLeftEncoderChannelA		= 0;
	const uint32_t frontLeftEncoderChannelB		= 1;
	const uint32_t frontRightEncoderChannelA	= 2;
	const uint32_t frontRightEncoderChannelB	= 3;
	const uint32_t backLeftEncoderChannelA		= 4;
	const uint32_t backLeftEncoderChannelB		= 5;
	const uint32_t backRightEncoderChannelA		= 6;
	const uint32_t backRightEncoderChannelB		= 7;


	//channels for analog sensors
	const uint32_t gyroChannel					= 0;
	const uint32_t gyroThermChannel				= 1;
	const uint32_t vacuumSensor1Channel			= 2;

	//channels for digital sensors

	//channels for other things
	const uint32_t joystickChannel				= 0;
	const uint32_t compressorChannel	    	= 0;

	//booleans for Autonomous
	int currentAutoOperation = 0;
	int count = 0;

	//PID Coefficients
	double gyroStraightP						= 0.5;
	double gyroStraightI						= 0.03;
	double gyroStraightD						= 0.1;

	double gyroStrafeP							= 0.001;
	double gyroStrafeI							= 0.000005;
	double gyroStrafeD							= 2000.0;

	double strafeP								= 0.0;
	double strafeI								= 0.2;
	double strafeD								= 0.0;

	double straightP							= 0.0;
	double straightI							= 0.2;
	double straightD							= 0.0;


	//inputs
	PowerDistributionPanel* pdp;
	Accelerometer* accelerometer;
	AnalogInput* therm;
	AnalogInput* gyro;
	AnalogInput* vacuumSensor1;
	Joystick* stick;			//currently the only joystick

	//Drive Motors
	Talon* frontLeftWheel;
	Talon* backLeftWheel;
	Talon* frontRightWheel;
	Talon* backRightWheel;

	//Drive Encoders
	Encoder* frontLeftEncoder;
	Encoder* frontRightEncoder;
	Encoder* backLeftEncoder;
	Encoder* backRightEncoder;

	//Other Outputs
	Talon* vacuumMotor1;
	Compressor* compressor;

	//Controlling Objects
	Vacuum* vacuum1;
	OI* opIn;
	MechanumDriveTrain* driveTrain;
	CorrectedGyro* roboGyro;
	DriveEncoders* driveEncoders;

	//PID Loops
	GyroPID* gyroPID;
	EncoderDrivePID* straightDrivePID;
	EncoderDrivePID* strafeDrivePID;

public:

	void RobotInit()
	{
		//inputs
		pdp = new PowerDistributionPanel();
		therm = new AnalogInput(gyroThermChannel);
		gyro = new AnalogInput(gyroChannel);
		stick = new Joystick(joystickChannel);
		accelerometer = new BuiltInAccelerometer();
		vacuumSensor1 = new AnalogInput(vacuumSensor1Channel);

		//Drive Motors
		frontLeftWheel = new Talon(frontLeftChannel);
		backLeftWheel = new Talon(backLeftChannel);
		frontRightWheel = new Talon(frontRightChannel);
		backRightWheel = new Talon(backRightChannel);

		//Drive Encoder
		frontLeftEncoder = new Encoder(frontLeftEncoderChannelA,frontLeftEncoderChannelB);
		frontRightEncoder = new Encoder(frontRightEncoderChannelA,frontRightEncoderChannelB);
		backLeftEncoder = new Encoder(backLeftEncoderChannelA,backLeftEncoderChannelB);
		backRightEncoder = new Encoder(backRightEncoderChannelA,backRightEncoderChannelA);

		//other outputs
		compressor = new Compressor(compressorChannel);
		vacuumMotor1 = new Talon(vacuumMotor1Channel);

		//wpilib class setup
		compressor->SetClosedLoopControl(true);

		//user generated classes
		vacuum1 = new Vacuum(vacuumSensor1, vacuumMotor1);
		opIn = new OI(stick);
		driveEncoders = new DriveEncoders(frontLeftEncoder, backLeftEncoder, frontRightEncoder, backRightEncoder);
		driveTrain = new MechanumDriveTrain(frontLeftWheel, backLeftWheel, frontRightWheel, backRightWheel, opIn,driveEncoders);
		roboGyro = new CorrectedGyro(gyro,therm);

		//PID Loops
		gyroPID = new GyroPID(gyroStraightP,gyroStraightI,gyroStraightD,roboGyro,driveTrain);
		straightDrivePID = new EncoderDrivePID(strafeP,strafeI,strafeD,driveEncoders,driveTrain,STRAIGHT);
		strafeDrivePID = new EncoderDrivePID(straightP,straightI,straightD,driveEncoders,driveTrain,STRAFE);

		//user generated classes setup
		double averageValue = (roboGyro->GetRaw() + roboGyro->GetRaw() + roboGyro->GetRaw()+ roboGyro->GetRaw()+ roboGyro->GetRaw())/5;
		roboGyro->SetZeroValue(averageValue);
		roboGyro->SetSensitivity(0.007);
		roboGyro->SetDeadband(4);
		roboGyro->Reset();
	}

	~Robot() {
		//inputs
		delete therm;
		delete gyro;
		delete stick;
		delete accelerometer;
		delete vacuumSensor1;

		//Drive Motors
		delete frontLeftWheel;
		delete backLeftWheel;
		delete frontRightWheel;
		delete backRightWheel;

		//Drive Encoders
		delete frontLeftEncoder;
		delete backLeftEncoder;
		delete frontRightEncoder;
		delete backRightEncoder;

		//outputs
		delete frontLeftWheel;
		delete backLeftWheel;
		delete frontRightWheel;
		delete backRightWheel;
		delete vacuumMotor1;
		delete compressor;

		//PID Loops
		delete gyroPID;
		delete straightDrivePID;
		delete strafeDrivePID;

		//user generated classes
		delete roboGyro;
		delete driveTrain;
		delete opIn;
		delete vacuum1;
	}

	void TestInit() {
		roboGyro->Reset();

	}

	void TestPeriodic() {
		SmartDashboard::PutNumber("GyroAngle",roboGyro->GetAngle());
	}


	/*
	 * preps for the human operated mode
	 */
	void TeleopInit() {
		printf("TeleopInit \n");
		roboGyro->Reset(); //resets the Gyro
		gyroPID->SetSetpoint(0.0); //sets the setpoint to zero
		compressor->SetClosedLoopControl(true);

	}

	/*
	 * human operated mode loop
	 */
	void TeleopPeriodic() {
		vacuum1->Start();
		gyroPID->Enable(); //enables PID
		printf("InTeleop\n");
		driveTrain->Drive(); //tells the robot to drive

		//put numbers to the smart dashboard for diagnostics
		//PDP Values from the drive
		SmartDashboard::PutNumber("FrontLeftMotorPower",pdp->GetCurrent(frontLeftPDPChannel)*pdp->GetVoltage());
		SmartDashboard::PutNumber("FrontRightMotorPower",pdp->GetCurrent(frontRightPDPChannel)*pdp->GetVoltage());
		SmartDashboard::PutNumber("BackLeftMotorPower",pdp->GetCurrent(backLeftPDPChannel)*pdp->GetVoltage());
		SmartDashboard::PutNumber("BackRightMotorPower",pdp->GetCurrent(backRightPDPChannel)*pdp->GetVoltage());
		//other information
		SmartDashboard::PutNumber("JoystickY",opIn->GetX());
		SmartDashboard::PutNumber("JoystickX",opIn->GetY());
		SmartDashboard::PutNumber("JoystickTwist",opIn->GetTwist());
		SmartDashboard::PutNumber("GyroAngle",roboGyro->GetAngle());
		SmartDashboard::PutNumber("GyroAngle",roboGyro->GetRate());
		SmartDashboard::PutNumber("JoystickThrottle",opIn->GetThrottle());
		SmartDashboard::PutBoolean("Compressor Enabled?",compressor->Enabled());
	}

	/*
	 * preps for autonomous mode
	 */
	void AutonomousInit() {
		roboGyro->Reset();
		gyroPID->SetSetpoint(0.0);
		gyroPID->Reset();
		strafeDrivePID->SetSetpoint(0.0);
		straightDrivePID->SetSetpoint(0.0);
		count = 0;
		//compressor->Enabled();
	}

	/*
	 * autonomous mode loop
	 */
	void AutonomousPeriodic() {

		SmartDashboard::PutNumber("GyroValue",gyro->GetValue());
		SmartDashboard::PutNumber("GyroAngle",roboGyro->GetAngle());
		gyroPID->Enable();
		switch(currentAutoOperation){
		case 0:
			gyroPID->SetPIDValues(gyroStraightP,gyroStraightI,gyroStraightD);
			driveTrain->DriveForward(.1*(count)/60.0); //needs to be changed to distances
			count++;
			if((count/20)==20)
			{
				count =0;
				currentAutoOperation++;
			}
			break;
		case 1:
			driveTrain->Stop();
			count++;
			if((count/20)==3)
			{
				count =0;
				currentAutoOperation++;
			}
			break;
		case 2:
			gyroPID->SetPIDValues(gyroStraightP,gyroStraightI,gyroStraightD);
			gyroPID->Reset();
			//driveTrain->DriveForward(.1*(count)/60.0); //needs to be changed to distances
			count++;
			if((count/20)==20)
			{
				count =0;
				currentAutoOperation++;
			}
			break;
		case 3:
			driveTrain->Stop();
			count++;
			if((count/20)==3)
			{
				count =0;
				currentAutoOperation++;
			}
			break;
		case 4:
			//driveTrain->DriveRight(.25); //needs to be changed to distances
			count++;
			if((count/20)==3)
			{
				count =0;
				currentAutoOperation++;
			}
			break;
		case 5:
			driveTrain->Stop();
			count++;
			if((count/20)==3)
			{
				count =0;
				currentAutoOperation++;
			}
			break;
		}
	}

	/*
	 * preps for switch to disabled
	 */
	void DisabledInit() {
		compressor->SetClosedLoopControl(false);
		gyroPID->Disable();
		gyroPID->Reset();
		straightDrivePID->Disable();
		straightDrivePID->Reset();
		strafeDrivePID->Disable();
		strafeDrivePID->Reset();
		driveTrain->Stop();
		gyroPID->Disable(); //stops the gyro PID
		vacuum1->Stop();
		currentAutoOperation = 0;
	}

};

START_ROBOT_CLASS(Robot);
