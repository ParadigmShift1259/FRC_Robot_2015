#include "WPILib.h"
#include "Joystick.h"

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive class.
 */
class Robot: public IterativeRobot
{

    // Channels for the wheels
	const uint32_t frontLeftChannel		= 0;
	const uint32_t backLeftChannel 		= 1;
	const uint32_t frontRightChannel 	= 2;
	const uint32_t backRightChannel 	= 3;


	uint32_t joystickChannel;
	Joystick stick;			// only joystick

public:

	Robot() :
		joystickChannel(0),
		stick(joystickChannel)
	{
	}

	void TeleopInit() {

	}

	void TelopPeriodic() {

	}

	void AutonomousInit() {

	}

	void AutonomousPeriodic() {

	}

};

START_ROBOT_CLASS(Robot);
