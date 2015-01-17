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


	uint32_t joystickChannel;
	Joystick stick;			// only joystick
	OI opIn;
	MechanumDriveTrain driveTrain;

	/*
	~Robot(){
		delete driveTrain;
		delete opIn;
	}
	*/


public:

	Robot() :
		joystickChannel(0),
		stick(joystickChannel),
		opIn(stick),
		driveTrain(frontLeftChannel, backLeftChannel,frontRightChannel,backRightChannel,opIn)
	{

	}

	void TeleopInit() {

	}

	void TelopPeriodic() {

		driveTrain.Drive();
		printf("test\n");
	}

	void AutonomousInit() {

	}

	void AutonomousPeriodic() {

	}

};

START_ROBOT_CLASS(Robot);
