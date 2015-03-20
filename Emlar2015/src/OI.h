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
	bool secondaryLastTriggered = false;
	bool secondaryTriggerState = false;
	bool button2LastCalled = false;
	bool button3LastCalled = false;
	bool button4LastCalled = false;
	bool button5LastCalled = false;
	bool button6LastCalled = false;
	bool button7LastCalled = false;
	bool secondaryButton2LastCalled = false;
	bool secondaryButton3LastCalled = false;
	bool secondaryButton4LastCalled = false;
	bool secondaryButton5LastCalled = false;
	bool secondaryButton6LastCalled = false;
	bool secondaryButton7LastCalled = false;
	bool secondaryButton8LastCalled = false;
	bool secondaryButton9LastCalled = false;
	bool secondaryButton10LastCalled = false;
	bool secondaryButton11LastCalled = false;
	bool secondaryButton12LastCalled = false;
	bool button2State = false;
	bool button3State = false;
	bool button4State = false;
	bool button5State = false;
	bool button6State = false;
	bool secondaryButton2State = false;
	bool secondaryButton3State = false;
	bool secondaryButton4State = false;
	bool secondaryButton5State = false;
	bool secondaryButton6State = false;

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
	bool GetToggleTrigger();
	bool GetSecondaryToggleTrigger();
	bool GetSingularSecondaryTrigger();
	double GetThrottle();
	double GetRawY();
	double GetSecondaryRawY();
	Joystick* GetJoystick() const;
	bool GetToggleButton2();
	bool GetToggleButton3();
	bool GetToggleButton4();
	bool GetSecondaryToggleButton2();
	bool GetSecondaryToggleButton3();
	bool GetSecondaryToggleButton4();
	bool GetSecondaryToggleButton5();
	bool GetRawButton2();
	bool GetRawButton3();
	bool GetRawButton4();
	bool GetSingularButton2();
	bool GetSingularButton3();
	bool GetSingularButton4();
	bool GetSingularButton5();
	bool GetSingularButton6();
	bool GetSingularButton7();
	bool GetSingularSecondaryButton2();
	bool GetSingularSecondaryButton3();
	bool GetSingularSecondaryButton4();
	bool GetSingularSecondaryButton5();
	bool GetSingularSecondaryButton6();
	bool GetSingularSecondaryButton7();
	bool GetSingularSecondaryButton8();
	bool GetSingularSecondaryButton9();
	bool GetSingularSecondaryButton10();
	bool GetSingularSecondaryButton11();
	bool GetSingularSecondaryButton12();
};

#endif /* SRC_OI_H_ */
