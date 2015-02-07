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
#include "Lifter.h"

/**
 * This is a demo program showing how to use Mechanum control with the RobotDrive class.
 */
class Robot: public IterativeRobot {

private:
	const double pi =
			3.141592653589793238462643383279502884197169399375105820974944592307816406286;

	//drive encoder information
	const double drivecpr = 360.0;
	const double gearRatio = 1;
	//final information const double gearRatio = 15.0/40.0;
	const double wheelDiameter = 6.0;
	const double rotationsPerInch = pi * wheelDiameter * gearRatio;
	const double inchesPerClickDrive = rotationsPerInch / drivecpr;

	//lifter encoder information
	const double liftercpr = 120.0;

	//Strafe and Straight Definitions
	static const int STRAIGHT = 1;
	static const int STRAFE = 2;

	//Lifter Position Definitions
	static const int FLOOR = 1;
	static const int TOTE = 2;
	static const int COOPSTEP = 3;
	static const int COOPSTEP1TOTE = 4;
	static const int COOPSTEP2TOTE = 5;
	static const int COOPSTEP3TOTE = 6;
	static const int SCORINGPLATFORM = 7;

	//Constants for Robot Properties
	//static const double distancePerTick		= ;
	//static const double rectOffset			= ;

	//PDP Channels
	const uint32_t frontLeftPDPChannel = 0;
	const uint32_t frontRightPDPChannel = 2;
	const uint32_t backLeftPDPChannel = 1;
	const uint32_t backRightPDPChannel = 3;

	//PWM Channels for the motors
	const uint32_t frontLeftChannel = 2;
	const uint32_t frontRightChannel = 0;
	const uint32_t backLeftChannel = 1;
	const uint32_t backRightChannel = 3;
	const uint32_t vacuumMotor1Channel = 4;

	//Drive Encoders
	const uint32_t frontLeftEncoderChannelA = 2;
	const uint32_t frontLeftEncoderChannelB = 3;
	const uint32_t frontRightEncoderChannelA = 0;
	const uint32_t frontRightEncoderChannelB = 1;
	const uint32_t backLeftEncoderChannelA = 6;
	const uint32_t backLeftEncoderChannelB = 7;
	const uint32_t backRightEncoderChannelA = 4;
	const uint32_t backRightEncoderChannelB = 5;

	//channels for analog sensors
	const uint32_t gyroChannel = 0;
	const uint32_t gyroThermChannel = 1;
	const uint32_t vacuumSensor1Channel = 2;

	//CAN Channels
	const uint32_t pcmChannel = 0;
	const uint32_t lifterCanChannel = 1;

	//piston channels
	const uint32_t toteGrabberChannelIn = 0;
	const uint32_t toteGrabberChannelOut = 1;
	const uint32_t toteDeployChannelIn = 2;
	const uint32_t toteDeployChannelOut = 3;
	const uint32_t vacuumDeployChannelIn = 4;
	const uint32_t vacuumDeployChannelOut = 5;

	//channels for other things
	const uint32_t joystickChannel = 0;

	//booleans for Autonomous
	int currentAutoOperation = 0;
	int count = 0;

	//Drivetrain temporary Storage
	double lastSetpoint;

	//PID Coefficients
	double gyroStraightP = 0.06;
	double gyroStraightI = 0.004;
	double gyroStraightD = 0.1;

	double gyroStrafeP = 0.001;
	double gyroStrafeI = 0.000005;
	double gyroStrafeD = 2000.0;

	double strafeP = 5.0;
	double strafeI = 0.1;
	double strafeD = 0.0;

	double straightP = 0.1;
	double straightI = 0.01;
	double straightD = 0.0;

	double lifterP = 0.1;
	double lifterI = 0.0;
	double lifterD = 0.0;

	//inputs
	PowerDistributionPanel* pdp;
	Accelerometer* accelerometer;
	AnalogInput* therm;
	AnalogInput* gyro;
	AnalogInput* vacuumSensor1;
	Joystick* stick;			//currently the only joystick

	//Lifter Motor
	CANTalon* lifterMotor;

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

	//Pistons
	DoubleSolenoid* toteGrabber;
	DoubleSolenoid* toteDeployer;
	DoubleSolenoid* vacuumDeployer;

	//Other Outputs
	Talon* vacuumMotor1;
	Compressor* compressor;

	//Controlling Objects
	Vacuum* vacuum1;
	OI* opIn;
	MecanumDriveTrain* driveTrain;
	DriveEncoders* driveEncoders;
	CorrectedGyro* roboGyro;
	Lifter* lifter;

	//PID Loops
	GyroPID* gyroPID;
	EncoderDrivePID* straightDrivePID;
	EncoderDrivePID* strafeDrivePID;

public:

	void RobotInit() {
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
		frontLeftEncoder = new Encoder(frontLeftEncoderChannelA,
				frontLeftEncoderChannelB);
		frontRightEncoder = new Encoder(frontRightEncoderChannelA,
				frontRightEncoderChannelB);
		backLeftEncoder = new Encoder(backLeftEncoderChannelA,
				backLeftEncoderChannelB);
		backRightEncoder = new Encoder(backRightEncoderChannelA,
				backRightEncoderChannelA);

		//Solenoids
		toteGrabber = new DoubleSolenoid(toteGrabberChannelOut,
				toteGrabberChannelIn);
		toteDeployer = new DoubleSolenoid(toteDeployChannelOut,
				toteDeployChannelIn);
		vacuumDeployer = new DoubleSolenoid(vacuumDeployChannelOut,
				vacuumDeployChannelIn);

		//Lifter Motor
		lifterMotor = new CANTalon(lifterCanChannel);

		//other outputs
		compressor = new Compressor(pcmChannel);
		vacuumMotor1 = new Talon(vacuumMotor1Channel);

		//wpilib class setup
		compressor->SetClosedLoopControl(true);
		frontLeftEncoder->SetDistancePerPulse(inchesPerClickDrive);
		backLeftEncoder->SetDistancePerPulse(inchesPerClickDrive);
		frontRightEncoder->SetDistancePerPulse(inchesPerClickDrive);
		backRightEncoder->SetDistancePerPulse(inchesPerClickDrive);

		//user generated classes
		vacuum1 = new Vacuum(vacuumSensor1, vacuumMotor1);
		opIn = new OI(stick);
		driveEncoders = new DriveEncoders(frontLeftEncoder, backLeftEncoder,
				frontRightEncoder, backRightEncoder);
		driveTrain = new MecanumDriveTrain(frontLeftWheel, backLeftWheel,
				frontRightWheel, backRightWheel, opIn);
		roboGyro = new CorrectedGyro(gyro, therm);
		lifter = new Lifter(lifterP, lifterI, lifterD, lifterMotor, toteGrabber,
				toteDeployer, vacuumDeployer, vacuum1);

		//user generated classes setup
		roboGyro->Reset();
		//PID Loops
		gyroPID = new GyroPID(gyroStraightP, gyroStraightI, gyroStraightD,
				roboGyro, driveTrain);
		straightDrivePID = new EncoderDrivePID(strafeP, strafeI, strafeD,
				driveEncoders, driveTrain, STRAIGHT);
		strafeDrivePID = new EncoderDrivePID(straightP, straightI, straightD,
				driveEncoders, driveTrain, STRAFE);
	}

	~Robot() {
		//inputs
		delete therm;
		delete gyro;
		delete stick;
		delete accelerometer;
		delete vacuumSensor1;

		//solenoids
		delete toteGrabber;
		delete toteDeployer;
		delete vacuumDeployer;

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

		//Lifter Motor
		delete lifterMotor;

		//outputs
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
		delete lifter;
	}

	void TestInit() {
		roboGyro->Reset();

	}

	void TestPeriodic() {
		SmartDashboard::PutNumber("GyroAngle", roboGyro->GetAngle());
	}

	/*
	 * preps for the human operated mode
	 */
	void TeleopInit() {
		printf("TeleopInit \n");
		roboGyro->Reset(); //resets the Gyro
		gyroPID->SetSetpoint(0.0); //sets the setpoint to zero
		compressor->SetClosedLoopControl(true);
		strafeDrivePID->SetSetpoint(0.0);
		straightDrivePID->SetSetpoint(0.0);
		gyroPID->Enable(); //enables PID
		gyroPID->SetSetpoint(0.0); //sets the setpoint to zero
	}

	/*
	 * human operated mode loop
	 */
	void TeleopPeriodic() {
		strafeDrivePID->Enable();

		straightDrivePID->Enable();
		vacuum1->Start();

		//driveTrain->DriveRight(opIn->GetX()); //tells the robot to drive
		printf("InTeleop\n");

		if (!driveTrain->GyroPIDDisabled()) {
			gyroPID->Enable();
			if (!opIn->GetTrigger()) {
				gyroPID->SetSetpoint(roboGyro->GetAngle());
				driveTrain->Drive();
			}
		}

		else {
			gyroPID->Disable();
			driveTrain->Turn(lastSetpoint + 180.0, roboGyro->GetAngle());
		}

		//gyroPID->SetSetpointRelative(opIn->GetTwist());
		if (opIn->GetButton2()) {
			lastSetpoint = roboGyro->GetAngle();
			gyroPID->Disable();
			gyroPID->SetSetpointRelative(180.0);
			driveTrain->EnableTurn();
		}

		//put numbers to the smart dashboard for diagnostics
		//PDP Values from the drive
		SmartDashboard::PutNumber("FrontLeftMotorPower",
				pdp->GetCurrent(frontLeftPDPChannel) * pdp->GetVoltage());
		SmartDashboard::PutNumber("FrontRightMotorPower",
				pdp->GetCurrent(frontRightPDPChannel) * pdp->GetVoltage());
		SmartDashboard::PutNumber("BackLeftMotorPower",
				pdp->GetCurrent(backLeftPDPChannel) * pdp->GetVoltage());
		SmartDashboard::PutNumber("BackRightMotorPower",
				pdp->GetCurrent(backRightPDPChannel) * pdp->GetVoltage());
		//other information
		SmartDashboard::PutNumber("JoystickY", opIn->GetX());
		SmartDashboard::PutNumber("JoystickX", opIn->GetY());
		SmartDashboard::PutNumber("JoystickTwist", opIn->GetTwist());
		SmartDashboard::PutNumber("GyroAngle", roboGyro->GetAngle());
		SmartDashboard::PutNumber("JoystickThrottle", opIn->GetThrottle());
		SmartDashboard::PutBoolean("Compressor Enabled?",
				compressor->Enabled());
		SmartDashboard::PutNumber("StraightDistance",
				driveEncoders->GetDistanceStraight());
		SmartDashboard::PutNumber("StrafeDistance",
				driveEncoders->GetDistanceStrafe());
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
		compressor->Enabled();
	}

	/*
	 * autonomous mode loop
	 */
	void AutonomousPeriodic() {

		SmartDashboard::PutNumber("GyroValue", gyro->GetValue());
		SmartDashboard::PutNumber("GyroAngle", roboGyro->GetAngle());
		gyroPID->Enable();
		switch (currentAutoOperation) {
		case 0:
			gyroPID->SetPIDValues(gyroStraightP, gyroStraightI, gyroStraightD);
			driveTrain->DriveForward(.1 * (count) / 60.0); //needs to be changed to distances
			count++;
			if ((count / 20) == 20) {
				count = 0;
				currentAutoOperation++;
			}
			break;
		case 1:
			driveTrain->Stop();
			count++;
			if ((count / 20) == 3) {
				count = 0;
				currentAutoOperation++;
			}
			break;
		case 2:
			gyroPID->SetPIDValues(gyroStraightP, gyroStraightI, gyroStraightD);
			gyroPID->Reset();
			driveTrain->DriveForward(.1 * (count) / 60.0); //needs to be changed to distances
			count++;
			if ((count / 20) == 20) {
				count = 0;
				currentAutoOperation++;
			}
			break;
		case 3:
			driveTrain->Stop();
			count++;
			if ((count / 20) == 3) {
				count = 0;
				currentAutoOperation++;
			}
			break;
		case 4:
			driveTrain->DriveRight(.25); //needs to be changed to distances
			count++;
			if ((count / 20) == 3) {
				count = 0;
				currentAutoOperation++;
			}
			break;
		case 5:
			driveTrain->Stop();
			count++;
			if ((count / 20) == 3) {
				count = 0;
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
		driveEncoders->ResetEncoders();
	}
};

START_ROBOT_CLASS(Robot);
