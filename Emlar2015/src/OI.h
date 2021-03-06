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

	const double shiftPerCycle = .01;

	const double deadzone = 0.1;
	const double twistDeadzone = 0.13;
	Joystick* mainJoystick;

public:
	OI(Joystick*);
	double GetX();
	double GetY();
	double GetTwist();
	bool GetTrigger();
	double GetThrottle();
	Joystick* GetJoystick() const;
	bool GetButton2();
};

#endif /* SRC_OI_H_ */
