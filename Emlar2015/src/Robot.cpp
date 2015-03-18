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
	const int numberOfVacuums = 5;
	const double pi =
			3.141592653589793238462643383279502884197169399375105820974944592307816406286;

	//drive encoder information
	double drivecpr = 360.0 * 4.0;
	double driveGearRatio = 1.0;
	double driveWheelDiameter = 6.0;
	double driveRotationsPerInch = pi * driveWheelDiameter * driveGearRatio;
	double driveInchesPerClick = driveRotationsPerInch / drivecpr;

	//lifter encoder information
	double liftercpr = 120.0 * 4.0;
	double lifterGearRatio = 1.0;
	double lifterSprocketDiameter = 2.406;
	double lifterInchesPerRotation = pi * lifterSprocketDiameter
			* lifterGearRatio;
	double lifterInchesPerClick = lifterInchesPerRotation / liftercpr;

	//Strafe and Straight Definitions
	static const int STRAIGHT = 1;
	static const int STRAFE = 2;

	//Lifter Position Definitions
	int FLOOR = 0.0 / lifterInchesPerClick; //make sure to change the ones in the lifter class
	int TOTE = 12.0 / lifterInchesPerClick; //make sure to change the ones in the lifter class
	int COOPSTEP = 6.25 / lifterInchesPerClick;
	int SCORINGPLATFORM = 2.0 / lifterInchesPerClick;
	int VACUUMCLEARANCE = 14.0 / lifterInchesPerClick;
	//Constants for Robot Properties
	//static const double distancePerTick		= ;
	//static const double rectOffset			= ;

	//PDP Channels
	const uint32_t frontLeftPDPChannel = 15;
	const uint32_t frontRightPDPChannel = 0;
	const uint32_t backLeftPDPChannel = 1;
	const uint32_t backRightPDPChannel = 0;

	//CAN Channels for the drive motors
	const uint32_t frontRightChannel = 1;
	const uint32_t backRightChannel = 2;
	const uint32_t backLeftChannel = 3;
	const uint32_t frontLeftChannel = 4;

	//PWM Channels for the intake wheels
	const uint32_t intakeWheelChannel = 0;

	//PWM Channels for the vacuums
	const uint32_t vacuumMotor1Channel = 1;
	const uint32_t vacuumMotor2Channel = 2;
	const uint32_t vacuumMotor3Channel = 3;
	const uint32_t vacuumMotor4Channel = 4;
	const uint32_t vacuumMotor5Channel = 5;

	//channels for analog sensors
	const uint32_t gyroChannel = 0;
	const uint32_t gyroThermChannel = 1;
	const uint32_t vacuumSensor1Channel = 2;

	//CAN Channels
	const uint32_t pcmChannel = 6;
	const uint32_t lifterCanChannel = 5;

	//piston channels
	const uint32_t toteDeployChannelIn = 0;
	const uint32_t toteDeployChannelOut = 1;
	const uint32_t toteGrabberChannelIn = 2;
	const uint32_t toteGrabberChannelOut = 3;
	const uint32_t vacuumDeployChannelIn = 4;
	const uint32_t vacuumDeployChannelOut = 5;

	//channels for other things
	const uint32_t driveJoystickChannel = 0;
	const uint32_t secondaryJoystickChannel = 1;

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

	double lifterP = 15.00;
	double lifterI = 0.005;
	double lifterD = 0.005;
	double lifterF = 0.00;

	double number = 0;

	//inputs
	PowerDistributionPanel* pdp;
	Accelerometer* accelerometer;
	AnalogInput* therm;
	AnalogInput* gyro;
	AnalogInput* vacuumSensor1;
	SPI* vacuumSPIBus;
	Joystick* driveJoystick;			//currently the only joystick
	Joystick* secondaryJoystick;

	//Lifter Motor
	CANTalon* lifterMotor;

	//Drive Motors
	CANTalon* frontLeftWheel;
	CANTalon* backLeftWheel;
	CANTalon* frontRightWheel;
	CANTalon* backRightWheel;

	//Pistons
	DoubleSolenoid* toteGrabber;
	DoubleSolenoid* toteDeployer;
	DoubleSolenoid* vacuumDeployer;

	//Intake Wheels
	Victor* intakeWheelVictor;

	//Other Outputs
	Talon* vacuumMotor1;
	Talon* vacuumMotor2;
	Talon* vacuumMotor3;
	Talon* vacuumMotor4;
	Talon* vacuumMotor5;
	Compressor* compressor;

	//Controlling Objects
	IntakeWheels* intakeWheels;
	VacuumSensors* vacuumSensors;
	Vacuum** vacuums;
	Vacuum* vacuum1;
	Vacuum* vacuum2;
	Vacuum* vacuum3;
	Vacuum* vacuum4;
	Vacuum* vacuum5;
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
		driveJoystick = new Joystick(driveJoystickChannel);
		secondaryJoystick = new Joystick(secondaryJoystickChannel);
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
		intakeWheelVictor = new Victor(intakeWheelChannel);
		printf("intake wheels made\n");
		printf("drive encoders made\n");

		//Solenoids
		toteGrabber = new DoubleSolenoid(pcmChannel, toteGrabberChannelOut,
				toteGrabberChannelIn);
		toteDeployer = new DoubleSolenoid(pcmChannel, toteDeployChannelOut,
				toteDeployChannelIn);
		vacuumDeployer = new DoubleSolenoid(pcmChannel, vacuumDeployChannelOut,
				vacuumDeployChannelIn);

		printf("solenoids made\n");

		//Lifter Motor
		lifterMotor = new CANTalon(lifterCanChannel);
		printf("lifter made\n");

		//other outputs
		compressor = new Compressor(pcmChannel);
		vacuumMotor1 = new Talon(vacuumMotor1Channel);
		vacuumMotor2 = new Talon(vacuumMotor2Channel);
		vacuumMotor3 = new Talon(vacuumMotor3Channel);
		vacuumMotor4 = new Talon(vacuumMotor4Channel);
		vacuumMotor5 = new Talon(vacuumMotor5Channel);
		printf("other outputs made\n");

		//wpilib class setup
		compressor->SetClosedLoopControl(true);
		frontRightWheel->ConfigEncoderCodesPerRev(driveInchesPerClick);
		backRightWheel->ConfigEncoderCodesPerRev(driveInchesPerClick);
		frontLeftWheel->ConfigEncoderCodesPerRev(driveInchesPerClick);
		backLeftWheel->ConfigEncoderCodesPerRev(driveInchesPerClick);

		frontRightWheel->SetFeedbackDevice(CANTalon::QuadEncoder);
		backRightWheel->SetFeedbackDevice(CANTalon::QuadEncoder);
		frontLeftWheel->SetFeedbackDevice(CANTalon::QuadEncoder);
		backLeftWheel->SetFeedbackDevice(CANTalon::QuadEncoder);

		frontRightWheel->ConfigLimitMode(
				CANTalon::kLimitMode_SrxDisableSwitchInputs);
		backRightWheel->ConfigLimitMode(
				CANTalon::kLimitMode_SrxDisableSwitchInputs);
		frontLeftWheel->ConfigLimitMode(
				CANTalon::kLimitMode_SrxDisableSwitchInputs);
		backLeftWheel->ConfigLimitMode(
				CANTalon::kLimitMode_SrxDisableSwitchInputs);

		lifterMotor->SetFeedbackDevice(CANTalon::QuadEncoder);
		lifterMotor->ConfigLimitMode(CANTalon::kLimitMode_SwitchInputsOnly);
		lifterMotor->ConfigReverseLimit(0.0);
		lifterMotor->ConfigForwardLimit(10.0);
		lifterMotor->ConfigEncoderCodesPerRev(liftercpr);
		lifterMotor->SetPosition(0.0);
		lifterMotor->ConfigReverseLimit(-1.0);
		lifterMotor->SetSensorDirection(true);
		lifterMotor->SetF(lifterF);
		frontLeftWheel->SetControlMode(CANTalon::kPercentVbus);
		backLeftWheel->SetControlMode(CANTalon::kPercentVbus);
		frontRightWheel->SetControlMode(CANTalon::kPercentVbus);
		backRightWheel->SetControlMode(CANTalon::kPercentVbus);

		printf("wpi classes setup done\n");

		//user generated classes
		printf("Made everything before user generated classes\n");
		intakeWheels = new IntakeWheels(intakeWheelVictor);
		vacuum1 = new Vacuum(vacuumMotor1);
		vacuum2 = new Vacuum(vacuumMotor2);
		vacuum3 = new Vacuum(vacuumMotor3);
		vacuum4 = new Vacuum(vacuumMotor4);
		vacuum5 = new Vacuum(vacuumMotor5);
		opIn = new OI(driveJoystick, secondaryJoystick);
		driveEncoders = new DriveEncoders(frontRightWheel, backRightWheel,
				frontLeftWheel, backLeftWheel);
		driveTrain = new MecanumDriveTrain(frontLeftWheel, backLeftWheel,
				frontRightWheel, backRightWheel, opIn, driveEncoders);
		roboGyro = new CorrectedGyro(gyro, therm);
		vacuums = new Vacuum*[numberOfVacuums] { vacuum1, vacuum2, vacuum3,
				vacuum4, vacuum5 };	//add the rest of the vaccums here and make sure number of vacuums is large enough
		vacuumSensors = new VacuumSensors(vacuumSPIBus);
		lifter = new Lifter(lifterP, lifterI, lifterD, lifterMotor, toteGrabber,
				toteDeployer, vacuumDeployer, vacuums, vacuumSensors,
				intakeWheels, numberOfVacuums);

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
		delete driveJoystick;
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

		//Lifter Motor
		delete lifterMotor;

		//outputs
		delete vacuumMotor1;
		delete vacuumMotor2;
		delete vacuumMotor3;
		delete vacuumMotor4;
		delete vacuumMotor5;
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
		delete vacuums;
		delete vacuum1;
		delete vacuum2;
		delete vacuum3;
		delete vacuum4;
		delete vacuum5;
		delete lifter;
		delete vacuumSensors;

		delete dataIn;
		delete dataOut;
	}

	void TestInit() {
		//roboGyro->Reset();
		number = 0;
		count = 0;
		lifterMotor->DisableSoftPositionLimits();
		lifterMotor->SetControlMode(CANTalon::kPercentVbus);
		lifter->ReleaseTote();
		//compressor->SetClosedLoopControl(true);
	}

	void TestPeriodic() {
		printf("lifter position = %i\n", lifterMotor->GetEncPosition());

		printf("lifter Height = %f\n", lifterMotor->GetPosition());
		printf("lifter setpoint = %f\n", lifterMotor->GetSetpoint());
		printf("Lifter upper limit = %i\n",
				lifterMotor->IsFwdLimitSwitchClosed());
		printf("Lifter upper limit = %i\n",
				lifterMotor->IsRevLimitSwitchClosed());
		//for (int i = 0; i<numberOfVacuums; i++) {
		//	vacuums[i]->Start();
		//}
		compressor->SetClosedLoopControl(true);
		//printf("%f\n", lifterMotor->GetOutputVoltage());
		lifterMotor->Set(opIn->GetRawY());
		if (opIn->GetToggleButton3()) {
			lifter->DeployTote();
		} else {
			lifter->RetractTote();
		}
		if (opIn->GetToggleButton2()) {
			lifter->DeployVacuum();
			intakeWheels->EnableWheels();
			lifter->StartVacuums();
		} else {
			lifter->RetractVacuum();
		}
		if (opIn->GetTrigger()) {
			lifter->StopVacuums();
			intakeWheels->DisableWheels();
		}
		if (opIn->GetToggleButton4()) {
			lifter->GrabTote();
		} else {
			lifter->ReleaseTote();
		}
		if (!lifter->Zeroed()) {
			lifter->Zero();
		} else {
			lifterMotor->SetControlMode(CANTalon::kPercentVbus);
		}
		SmartDashboard::PutBoolean("LowerLimitZeroed",lifterMotor->IsRevLimitSwitchClosed());
		SmartDashboard::PutBoolean("UpperLimitZeroed",lifterMotor->IsFwdLimitSwitchClosed());
		//lifter->StartVacuums();
		//printf("Button2=%i\n",opIn->GetToggleButton2());
		//printf("Button3=%i\n",opIn->GetToggleButton3());
		//printf("Button4=%i\n",opIn->GetToggleButton4());
		//printf("NumberOfVacuumsAttached%i\n",vacuumSensors->IsAttached());
		SmartDashboard::PutBoolean("SafeToDeployVacuum",
				lifter->SafeChangeVacuumState());

	}

	/*
	 * preps for the human operated mode
	 */
	void TeleopInit() {
		printf("TeleopInit \n");
		//lifter->RetractVacuum();
		//lifter->RetractTote();
		//lifter->RetractTote();
		roboGyro->Reset(); //resets the Gyro
		gyroPID->SetSetpoint(0.0); //sets the setpoint to zero
		compressor->SetClosedLoopControl(true);
		//strafeDrivePID->SetSetpoint(0.0);
		//straightDrivePID->SetSetpoint(0.0);
		gyroPID->Enable(); //enables PID
		gyroPID->SetSetpoint(0.0); //sets the setpoint to zero
		lifterMotor->SetControlMode(CANTalon::kPosition);
		lifterMotor->EnableControl();
		//lifter->GrabTote();
		//lifter->RetractTote();
		lifterMotor->SetPosition(-1);
		lifter->ReleaseTote();
	}

	/*
	 * human operated mode loop
	 */
	void TeleopPeriodic() {
		compressor->SetClosedLoopControl(true);
		printf("In Teleop\n");
		//lifter->AutoGrabTote();
		//lifter->ContinueDrop();
		//strafeDrivePID->Enable();
		//straightDrivePID->Enable();

		printf("lifter Height = %f\n",
				lifterMotor->GetPosition() * lifterInchesPerClick);
		printf("lifter setpoint = %f\n", lifterMotor->GetSetpoint());
		printf("Lifter upper limit = %i\n",
				lifterMotor->IsFwdLimitSwitchClosed());
		printf("Lifter upper limit = %i\n",
				lifterMotor->IsRevLimitSwitchClosed());
		if (lifter->Zeroed()) {
			lifter->LifterQueuedFunctions();
			if (opIn->GetSingularSecondaryButton2()) {
				lifterMotor->Set(COOPSTEP);
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
			if (opIn->GetSingularSecondaryButton3()) {
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
			if (opIn->GetSingularSecondaryButton4()) {
				lifter->Zero();
			}
			if (opIn->GetSingularSecondaryButton5()) {
				lifter->BeginAutoGrabTote();
			}
			if (opIn->GetSingularSecondaryButton7()){
				lifter->SkipVacuumSensors();
			}
			if (!(lifter->GrabbingTote() || lifter->DroppingTote())) {
				lifter->MoveTo(VACUUMCLEARANCE);
				lifter->RetractVacuum();
			}
		} else {
			lifter->Zero();
		}
		if (opIn->GetSingularButton6()) {
			lifter->Drop();
		}
		printf("Encoder Position = %i\n", lifterMotor->GetEncPosition());
		printf("Lifter Setpoint = %f\n", lifterMotor->GetSetpoint());

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
		if (opIn->GetSingularButton2()) {
			lastSetpoint = roboGyro->GetAngle();
			gyroPID->Disable();
			gyroPID->SetSetpointRelative(180.0);
			driveTrain->EnableTurn();
		}
		/*
		 if (opIn->GetButton3()) {
		 vacuum1->Start();
		 } else {
		 vacuum1->Stop();
		 }
		 */
		//put numbers to the smart dashboard for diagnostics
		//PDP Values from the drive
		SmartDashboard::PutBoolean("LowerLimitZeroed",lifterMotor->IsRevLimitSwitchClosed());
		SmartDashboard::PutBoolean("UpperLimitZeroed",lifterMotor->IsFwdLimitSwitchClosed());

		SmartDashboard::PutNumber("CH0", vacuumSensors->GetCH0());
		SmartDashboard::PutNumber("CH1", vacuumSensors->GetCH1());
		SmartDashboard::PutNumber("CH2", vacuumSensors->GetCH2());
		SmartDashboard::PutNumber("CH3", vacuumSensors->GetCH3());
		SmartDashboard::PutNumber("CH4", vacuumSensors->GetCH4());

		SmartDashboard::PutNumber("GyroAngle", roboGyro->GetAngle());
		SmartDashboard::PutBoolean("Compressor Enabled?",
				compressor->Enabled());
		SmartDashboard::PutNumber("StraightDistance",
				driveEncoders->GetDistanceStraight());
		SmartDashboard::PutNumber("StrafeDistance",
				driveEncoders->GetDistanceStrafe());
		SmartDashboard::PutBoolean("SafeToDeployVacuum",
				lifter->SafeChangeVacuumState());
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
			if (!driveTrain->DriveForward((2.0 * 12.0 + 9.0)/ driveInchesPerClick)) {		//inches
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
			if (!driveTrain->DriveForward((2.0 * 12.0 + 9.0)/ driveInchesPerClick)) {		//inches
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
			if (!driveTrain->DriveRight((8.0 * 12.0 + 11.0)/ driveInchesPerClick)) {		//inches
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
		lifter->Disable();
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

START_ROBOT_CLASS (Robot);
