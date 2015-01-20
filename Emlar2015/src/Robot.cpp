#include "WPILib.h"
#include "Joystick.h"
#include "OI.h"
#include "MechanumDriveTrain.h"
#include <stdio.h>
/**
 * This is a demo program showing how to use Mechanum control with the RobotDrive class.
 */
class Robot: public IterativeRobot
{
private:
    // Channels for the wheels
	const uint32_t frontLeftChannel		= 2;
	const uint32_t backLeftChannel 		= 1;
	const uint32_t frontRightChannel 	= 0;
	const uint32_t backRightChannel 	= 3;

	const uint32_t joystickChannel		= 0;

	Talon* frontLeftWheel;
	Talon* backLeftWheel;
	Talon* frontRightWheel;
	Talon* backRightWheel;

	Joystick* stick;			// only joystick
	OI* opIn;
	MechanumDriveTrain* driveTrain;

	/*
	~Robot(){
		delete driveTrain;
		delete opIn;
	}
	*/


public:

	void RobotInit()
	{
		stick = new Joystick(joystickChannel);
		opIn = new OI(stick);
		frontLeftWheel = new Talon(frontLeftChannel);
		backLeftWheel = new Talon(backLeftChannel);
		frontRightWheel = new Talon(frontRightChannel);
		backRightWheel = new Talon(backRightChannel);
		driveTrain = new MechanumDriveTrain(frontLeftWheel, backLeftWheel,frontRightWheel,backRightWheel,opIn);

	}

	~Robot() {
		delete driveTrain;
		delete opIn;
		delete stick;
		delete frontLeftWheel;
		delete backLeftWheel;
		delete frontRightWheel;
		delete backRightWheel;
	}

	void TeleopInit() {

	}

	void TeleopPeriodic() {
		driveTrain->Drive();
		SmartDashboard::PutNumber("JoystickY",opIn->GetX());
		SmartDashboard::PutNumber("JoystickX",opIn->GetY());
		SmartDashboard::PutNumber("JoystickTwist",opIn->GetTwist());
	}

	void AutonomousInit() {

	}

	void AutonomousPeriodic() {
		std::cout<<"In Autonomous"<<std::endl;
	}

};

START_ROBOT_CLASS(Robot);
