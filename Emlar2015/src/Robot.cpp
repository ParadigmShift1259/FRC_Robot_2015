#include <MecanumDriveTrain.h>
#include "WPILib.h"
#include "Joystick.h"
#include "OI.h"
#include <stdio.h>
#include "GyroPID.h"
#include "CorrectedGyro.h"
#include "AccelPID.h"
#include "Vacuum.h"

/**
 * This is a demo program showing how to use Mechanum control with the RobotDrive class.
 */
class Robot: public IterativeRobot
{

private:
	//Axis definitions
	static const int X						= 1;
	static const int Y						= 2;
	static const int Z						= 3;

	//PDP Channels
	const uint32_t frontLeftPDPChannel		= 0;
	const uint32_t frontRightPDPChannel		= 2;
	const uint32_t backLeftPDPChannel		= 1;
	const uint32_t backRightPDPChannel		= 3;

    // Channels for the motors
	const uint32_t frontLeftChannel			= 2;
	const uint32_t frontRightChannel 		= 0;
	const uint32_t backLeftChannel 			= 1;
	const uint32_t backRightChannel 		= 3;
	const uint32_t vacuumMotor1Channel		= 4;

	//Drive Encoders
	const uint32_t frontLeftEncoderChannel	= 0;
	const uint32_t frontRightEncoderChannel	= 1;
	const uint32_t backLeftEncoderChannel	= 2;
	const uint32_t backRightEncoderChannel	= 3;


	//channels for analog sensors
	const uint32_t gyroChannel				= 0;
	const uint32_t gyroThermChannel			= 1;
	const uint32_t vacuumSensor1Channel		= 2;

	//channels for digital sensors

	//channels for other things
	const uint32_t joystickChannel		= 0;
	const uint32_t compressorChannel    = 0;

	//booleans for Autonomous
	int currentAutoOperation = 0;
	int count = 0;

	//PID Coefficients
	double gyroStraightP = 0.5;
	double gyroStraightI = 0.03;
	double gyroStraightD = 0.1;

	double gyroStraifP = 0.001;
	double gyroStraifI = 0.000005;
	double gyroStraifD = 2000.0;

	double accelStraifP = 0.0;
	double accelStraifI = -0.2;
	double accelStraifD = 0.0;

	double accelStraightP = 0.0;
	double accelStraightI = -0.2;
	double accelStraightD = 0.0;


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

	//PID Loops
	GyroPID* gyroPID;
	AccelPID* xAccelPID;
	AccelPID* yAccelPID;

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
		frontLeftEncoder = new Encoder(frontLeftEncoderChannel);
		frontRightEncoder = new Encoder(frontRightEncoderChannel);
		backLeftEncoder = new Encoder(backLeftEncoderChannel);
		backRightEncoder = new Encoder(backRightEncoderChannel);

		//other outputs
		compressor = new Compressor(compressorChannel);
		vacuumMotor1 = new Talon(vacuumMotor1Channel);

		//wpilib class setup
		compressor->SetClosedLoopControl(true);

		//user generated classes
		vacuum1 = new Vacuum(vacuumSensor1, vacuumMotor1);
		opIn = new OI(stick);
		driveTrain = new MechanumDriveTrain(frontLeftWheel, backLeftWheel, frontRightWheel, backRightWheel, frontLeftEncoder, backLeftEncoder, frontRightEncoder, backRightEncoder, opIn);
		roboGyro = new CorrectedGyro(gyro,therm);

		//PID Loops
		gyroPID = new GyroPID(gyroStraightP,gyroStraightI,gyroStraightD,roboGyro,driveTrain);
		xAccelPID = new AccelPID(accelStraifP,accelStraifI,accelStraifD,accelerometer,driveTrain,X);
		yAccelPID = new AccelPID(accelStraightP,accelStraightI,accelStraightD,accelerometer,driveTrain,Y);

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
		delete xAccelPID;
		delete yAccelPID;

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
		xAccelPID->SetSetpoint(0.0);
		yAccelPID->SetSetpoint(0.0);
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
		xAccelPID->Disable();
		xAccelPID->Reset();
		yAccelPID->Disable();
		yAccelPID->Reset();
		driveTrain->Stop();
		gyroPID->Disable(); //stops the gyro PID
		vacuum1->Stop();
		currentAutoOperation = 0;
	}

};

START_ROBOT_CLASS(Robot);
