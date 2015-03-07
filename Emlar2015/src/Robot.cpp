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
#include "VacuumSensors.h"
#include "IntakeWheels.h"

/**
 * This is a demo program showing how to use Mechanum control with the RobotDrive class.
 */
class Robot: public IterativeRobot {

private:
	const int numberOfVacuums = 1;
	const double pi =
			3.141592653589793238462643383279502884197169399375105820974944592307816406286;

	//drive encoder information
	const double drivecpr = 360.0;
	const double driveGearRatio = 1.0;
	//final information const double gearRatio = 15.0/40.0;
	const double driveWheelDiameter = 6.0;
	const double driveRotationsPerInch = pi * driveWheelDiameter
			* driveGearRatio;
	const double driveInchesPerClick = driveRotationsPerInch / drivecpr;

	//lifter encoder information
	const double liftercpr = 120.0;
	const double lifterGearRatio = 1.0;
	const double lifterSprocketDiameter = 1.0;
	const double lifterRotationsPerInch = pi * lifterSprocketDiameter
			* lifterGearRatio;
	const double lifterInchesPerClick = lifterRotationsPerInch / liftercpr;

	//Strafe and Straight Definitions
	static const int STRAIGHT = 1;
	static const int STRAFE = 2;

	//Lifter Position Definitions
	const int FLOOR = 0.0 / lifterRotationsPerInch; //make sure to change the ones in the lifter class
	const int TOTE = 12.0 / lifterRotationsPerInch; //make sure to change the ones in the lifter class
	const int COOPSTEP = 6.25 / lifterRotationsPerInch;
	const int SCORINGPLATFORM = 2.0 / lifterRotationsPerInch;

	//Constants for Robot Properties
	//static const double distancePerTick		= ;
	//static const double rectOffset			= ;

	//PDP Channels
	const uint32_t frontLeftPDPChannel = 2;
	const uint32_t frontRightPDPChannel = 0;
	const uint32_t backLeftPDPChannel = 1;
	const uint32_t backRightPDPChannel = 3;

	//CAN Channels for the drive motors
	const uint32_t frontLeftChannel = 0;
	const uint32_t frontRightChannel = 1;
	const uint32_t backLeftChannel = 2;
	const uint32_t backRightChannel = 3;

	//PWM Channels for the intake wheels
	const uint32_t leftIntakeWheelChannel = 4;
	const uint32_t rightIntakeWheelChannel = 5;

	//PWM Channels for the vacuums
	const uint32_t vacuumMotor1Channel = 6;

	//Drive Encoders
	const uint32_t frontLeftEncoderChannelA = 0;
	const uint32_t frontLeftEncoderChannelB = 1;
	const uint32_t frontRightEncoderChannelA = 2;
	const uint32_t frontRightEncoderChannelB = 3;
	const uint32_t backLeftEncoderChannelA = 4;
	const uint32_t backLeftEncoderChannelB = 5;
	const uint32_t backRightEncoderChannelA = 6;
	const uint32_t backRightEncoderChannelB = 7;

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

	double number = 0;

	//inputs
	PowerDistributionPanel* pdp;
	Accelerometer* accelerometer;
	AnalogInput* therm;
	AnalogInput* gyro;
	AnalogInput* vacuumSensor1;
	SPI* vacuumSPIBus;
	Joystick* stick;			//currently the only joystick

	//Lifter Motor
	CANTalon* lifterMotor;

	//Drive Motors
	CANTalon* frontLeftWheel;
	CANTalon* backLeftWheel;
	CANTalon* frontRightWheel;
	CANTalon* backRightWheel;

	//Drive Encoders
	Encoder* frontLeftEncoder;
	Encoder* frontRightEncoder;
	Encoder* backLeftEncoder;
	Encoder* backRightEncoder;

	//Pistons
	DoubleSolenoid* toteGrabber;
	DoubleSolenoid* toteDeployer;
	DoubleSolenoid* vacuumDeployer;

	//Intake Wheels
	Talon* leftIntakeWheel;
	Talon* rightIntakeWheel;

	//Other Outputs
	Talon* vacuumMotor1;
	Compressor* compressor;

	//Controlling Objects
	IntakeWheels* intakeWheels;
	VacuumSensors* vacuumSensors;
	Vacuum* vacuums;
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

	uint8_t* dataOut;
	uint8_t* dataIn;

public:

	void RobotInit() {
		//inputs
		printf("Robot init\n");
		pdp = new PowerDistributionPanel();
		therm = new AnalogInput(gyroThermChannel);
		gyro = new AnalogInput(gyroChannel);
		stick = new Joystick(joystickChannel);
		accelerometer = new BuiltInAccelerometer();
		vacuumSensor1 = new AnalogInput(vacuumSensor1Channel);
		vacuumSPIBus = new SPI(SPI::kMXP);
		printf("inputs made\n");

		//Drive Motors
		frontLeftWheel = new CANTalon(frontLeftChannel);
		backLeftWheel = new CANTalon(backLeftChannel);
		frontRightWheel = new CANTalon(frontRightChannel);
		backRightWheel = new CANTalon(backRightChannel);

		printf("drive motors made\n");

		//Intake Wheels
		leftIntakeWheel = new Talon(leftIntakeWheelChannel);
		rightIntakeWheel = new Talon(rightIntakeWheelChannel);
		printf("intake wheels made\n");

		//Drive Encoder
		frontLeftEncoder = new Encoder(frontLeftEncoderChannelA,
				frontLeftEncoderChannelB);
		frontRightEncoder = new Encoder(frontRightEncoderChannelA,
				frontRightEncoderChannelB);
		backLeftEncoder = new Encoder(backLeftEncoderChannelA,
				backLeftEncoderChannelB);
		backRightEncoder = new Encoder(backRightEncoderChannelA,
				backRightEncoderChannelA);

		printf("drive encoders made\n");

		//Solenoids
		toteGrabber = new DoubleSolenoid(toteGrabberChannelOut,
				toteGrabberChannelIn);
		toteDeployer = new DoubleSolenoid(toteDeployChannelOut,
				toteDeployChannelIn);
		vacuumDeployer = new DoubleSolenoid(vacuumDeployChannelOut,
				vacuumDeployChannelIn);

		printf("solenoids made\n");

		//Lifter Motor
		lifterMotor = new CANTalon(lifterCanChannel);
		printf("lifter made\n");

		//other outputs
		compressor = new Compressor(pcmChannel);
		vacuumMotor1 = new Talon(vacuumMotor1Channel);
		printf("other outputs made\n");

		//wpilib class setup
		compressor->SetClosedLoopControl(true);
		frontLeftEncoder->SetDistancePerPulse(driveInchesPerClick);
		backLeftEncoder->SetDistancePerPulse(driveInchesPerClick);
		frontRightEncoder->SetDistancePerPulse(driveInchesPerClick);
		backRightEncoder->SetDistancePerPulse(driveInchesPerClick);
		lifterMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
		lifterMotor->ConfigLimitMode(CANTalon::kLimitMode_SwitchInputsOnly);
		lifterMotor->ConfigEncoderCodesPerRev(liftercpr);
		backLeftWheel->SetControlMode(CANTalon::kSpeed);
		printf("wpi classes setup done\n");

		//user generated classes
		printf("Made everything before user generated classes\n");
		intakeWheels = new IntakeWheels(leftIntakeWheel, rightIntakeWheel);
		vacuum1 = new Vacuum(vacuumMotor1);
		opIn = new OI(stick);
		driveEncoders = new DriveEncoders(frontLeftEncoder, backLeftEncoder,
				frontRightEncoder, backRightEncoder);
		driveTrain = new MecanumDriveTrain(frontLeftWheel, backLeftWheel,
				frontRightWheel, backRightWheel, opIn, driveEncoders);
		roboGyro = new CorrectedGyro(gyro, therm);
		vacuums = {vacuum1};//add the rest of the vaccums here and make sure number of vacuums is large enough
		lifter = new Lifter(lifterP, lifterI, lifterD, lifterMotor, toteGrabber,
				toteDeployer, vacuumDeployer, vacuums, vacuumSensors,
				intakeWheels, numberOfVacuums);
		vacuumSensors = new VacuumSensors(vacuumSPIBus);

		printf("user generated classes made\n");
		//user generated classes setup
		roboGyro->Reset();
		//PID Loops
		gyroPID = new GyroPID(gyroStraightP, gyroStraightI, gyroStraightD,
				roboGyro, driveTrain);
		straightDrivePID = new EncoderDrivePID(strafeP, strafeI, strafeD,
				driveEncoders, driveTrain, STRAIGHT);
		strafeDrivePID = new EncoderDrivePID(straightP, straightI, straightD,
				driveEncoders, driveTrain, STRAFE);

		vacuumSPIBus->SetClockRate(50000);
		vacuumSPIBus->SetMSBFirst();
		vacuumSPIBus->SetSampleDataOnRising();
		vacuumSPIBus->SetClockActiveHigh();
		vacuumSPIBus->SetChipSelectActiveLow();

		printf("setup everything\n");
	}

	~Robot() {
		//inputs
		delete therm;
		delete gyro;
		delete stick;
		delete accelerometer;
		delete vacuumSensor1;
		delete vacuumSPIBus;

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
		delete intakeWheels;
		delete roboGyro;
		delete driveTrain;
		delete opIn;
		delete vacuum1;
		delete lifter;
		delete vacuumSensors;

		delete dataIn;
		delete dataOut;
	}

	void TestInit() {
		//roboGyro->Reset();
		number = 0;
		count = 0;
	}

	void TestPeriodic() {
		printf("CH 0 vacuum sensors= %u \n", vacuumSensors->GetCH0());
	}

	/*
	 * preps for the human operated mode
	 */
	void TeleopInit() {
		printf("TeleopInit \n");
		roboGyro->Reset(); //resets the Gyro
		gyroPID->SetSetpoint(0.0); //sets the setpoint to zero
		compressor->SetClosedLoopControl(false);
		strafeDrivePID->SetSetpoint(0.0);
		straightDrivePID->SetSetpoint(0.0);
		gyroPID->Enable(); //enables PID
		gyroPID->SetSetpoint(0.0); //sets the setpoint to zero
		//lifter->GrabTote();
		//lifter->RetractTote();
	}

	/*
	 * human operated mode loop
	 */
	void TeleopPeriodic() {
		printf("In Teleop\n");
		//lifter->AutoGrabTote();
		//lifter->ContinueDrop();
		strafeDrivePID->Enable();
		straightDrivePID->Enable();
		if (lifter->Zeroed()) {
			lifter->LifterQueuedFunctions();
			if (false) {
				lifter->MoveTo(COOPSTEP);
			}
			if (false) {
				lifter->MoveTo(COOPSTEP + TOTE);
			}
			if (false) {
				lifter->MoveTo(COOPSTEP + TOTE + TOTE);
			}
			if (false) {
				lifter->MoveTo(COOPSTEP + TOTE + TOTE + TOTE);
			}
			if (false) {
				lifter->MoveTo(SCORINGPLATFORM);
			}
			if (false) {
				lifter->MoveTo(SCORINGPLATFORM + TOTE);
			}
			if (false) {
				lifter->MoveTo(SCORINGPLATFORM + TOTE + TOTE);
			}
			if (false) {
				lifter->MoveTo(SCORINGPLATFORM + TOTE + TOTE + TOTE);
			}
			if (false) {
				lifter->Zero();
			}
		} else {
			lifter->Zero();
		}
		if (false) {
			lifter->Drop();
		}
		if (!driveTrain->GyroPIDDisabled()) {
			gyroPID->Enable();
			if (!opIn->GetTrigger()) {
				gyroPID->SetSetpoint(roboGyro->GetAngle());
			}
			driveTrain->Drive();
		} else {
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
		if (opIn->GetButton3()) {
			vacuum1->Start();
		} else {
			vacuum1->Stop();
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
			lifter->Zero();
			if (lifter->Zeroed()) {
				currentAutoOperation++;
			}
			break;
		case 1:
			driveTrain->Stop();
			lifter->BeginAutoGrabTote();
			if (!lifter->GrabbingTote()) {
				currentAutoOperation++;
			}
			break;
		case 2:
			gyroPID->SetPIDValues(gyroStraightP, gyroStraightI, gyroStraightD);
			if (!driveTrain->DriveForward(2.0 * 12.0 + 9.0)) {		//inches
				currentAutoOperation++;
			}
			break;
		case 3:
			driveTrain->Stop();
			if (count == 0) {
				lifter->BeginAutoGrabTote();
				count++;
			}
			if (!lifter->GrabbingTote()) {
				currentAutoOperation++;
				count = 0;
			}
			break;
		case 4:
			gyroPID->SetPIDValues(gyroStraightP, gyroStraightI, gyroStraightD);
			gyroPID->Reset();
			if (!driveTrain->DriveForward(2.0 * 12.0 + 9.0)) {		//inches
				currentAutoOperation++;
			}
			break;
		case 5:
			driveTrain->Stop();
			if (count == 0) {
				lifter->BeginAutoGrabTote();
				count++;
			}
			if (!lifter->GrabbingTote()) {
				currentAutoOperation++;
				count = 0;
			}
			break;
		case 6:
			gyroPID->SetPIDValues(gyroStrafeP, gyroStrafeI, gyroStrafeD);
			gyroPID->Reset();
			if (!driveTrain->DriveRight(8.0 * 12.0 + 11.0)) {		//inches
				currentAutoOperation++;
			}
			break;
		case 7:
			driveTrain->Stop();
			count++;
			if ((count / 20.0) == 1.0) {
				count = 0;
				currentAutoOperation++;
			}
			break;
		case 8:
			lifter->MoveTo(FLOOR);
			if (lifter->InPos()) {
				currentAutoOperation++;
			}
			break;
		case 9:
			lifter->DeployTote();
			if ((count / 20.0 == 1.0)) {
				count = 0;
				currentAutoOperation++;
			}
			break;
		case 10:
			lifter->ReleaseTote();
			currentAutoOperation++;
			break;
		default:
		break;
		}
		lifter->LifterQueuedFunctions();
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
