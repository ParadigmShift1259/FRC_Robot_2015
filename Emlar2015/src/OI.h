/*
 * OI.h
 */
#include "WPILib.h"

#ifndef SRC_OI_H_
#define SRC_OI_H_

class OI {
private:
	double y = 0;
	double x = 0;
	double twist = 0;
	bool lastTriggered = false;
	bool triggerState = false;
	bool button2LastCalled = false;
	bool button3LastCalled = false;
	bool button4LastCalled = false;
	bool button5LastCalled = false;
	bool button6LastCalled = false;
	bool button2SecondaryLastCalled = false;
	bool button3SecondaryLastCalled = false;
	bool button4SecondaryLastCalled = false;
	bool button5SecondaryLastCalled = false;
	bool button6SecondaryLastCalled = false;
	bool button2State = false;
	bool button3State = false;
	bool button4State = false;
	bool button5State = false;
	bool button6State = false;

	const double shiftPerCycle = .1;

	const double deadzone = 0.15;
	const double twistDeadzone = 0.13;
	Joystick* driveJoystick;
	Joystick* secondaryJoystick;

public:
	OI(Joystick*,Joystick*);
	double GetX();
	double GetY();
	double GetTwist();
	bool GetTrigger();
	double GetThrottle();
	double GetRawY();
	double GetSecondaryRawY();
	Joystick* GetJoystick() const;
	bool GetToggleButton2();
	bool GetToggleButton3();
	bool GetToggleButton4();
	bool GetRawButton2();
	bool GetRawButton3();
	bool GetRawButton4();
	bool GetSingularButton2();
	bool GetSingularButton3();
	bool GetSingularButton4();
	bool GetSingularButton5();
	bool GetSingularButton6();
	bool GetSingularSecondaryButton2();
	bool GetSingularSecondaryButton3();
	bool GetSingularSecondaryButton4();
	bool GetSingularSecondaryButton5();
	bool GetSingularSecondaryButton6();
};

#endif /* SRC_OI_H_ */
