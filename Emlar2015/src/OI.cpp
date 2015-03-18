#include <Joystick.h>
#include <OI.h>
#include <cmath>

/*

 * OI.cpp
 *
 *  Created on: Jan 16, 2015
 *      Author: Programming
 */

OI::OI(Joystick* mainJoystick, Joystick* secondaryJoystick) {
	this->driveJoystick = mainJoystick;
	this->secondaryJoystick = secondaryJoystick;

}
double OI::GetX() {
	double currentX = (driveJoystick->GetX() / OI::GetThrottle());
	//adds deadzone and ramping
	if (currentX > deadzone || currentX < -deadzone) {
		if (std::abs(currentX - x) < shiftPerCycle) {
			x = currentX;
		} else if (currentX > x) {
			x = x + shiftPerCycle;
		} else {
			x = x - shiftPerCycle;
		}
	} else {
		currentX = 0;
		if (std::abs(currentX - x) < shiftPerCycle) {
			x = currentX;
		} else if (currentX > x) {
			x = x + shiftPerCycle;
		} else {
			x = x - shiftPerCycle;
		}
	}
	return x;
}
double OI::GetRawY() {
	return driveJoystick->GetY();
}
double OI::GetSecondaryRawY() {
	return secondaryJoystick->GetY();
}

double OI::GetY() {
	double currentY = (driveJoystick->GetY() / OI::GetThrottle());
	//adds deadzone and ramping
	if (currentY > deadzone || currentY < -deadzone) {
		if (std::abs(currentY - y) < shiftPerCycle) {
			y = currentY;
		} else if (currentY > y) {
			y = y + shiftPerCycle;
		} else {
			y = y - shiftPerCycle;
		}
	} else {
		currentY = 0;
		if (std::abs(currentY - y) < shiftPerCycle) {
			y = currentY;
		} else if (currentY > y) {
			y = y + shiftPerCycle;
		} else {
			y = y - shiftPerCycle;
		}
	}
	return y;
}

double OI::GetTwist() {
	double currentTwist = (driveJoystick->GetTwist() / OI::GetThrottle());
	//adds deadzone
	if (currentTwist > twistDeadzone || currentTwist < -twistDeadzone) {
		twist = currentTwist;
	} else {
		twist = 0;
	}
	return twist;
}

bool OI::GetToggleButton2() {
	if (!button2LastCalled && driveJoystick->GetRawButton(2)) {
		button2State = !button2State;
	}
	button2LastCalled = driveJoystick->GetRawButton(2);
	return button2State;
}

bool OI::GetRawButton2() {
	return driveJoystick->GetRawButton(2);
}
bool OI::GetRawButton3() {
	return driveJoystick->GetRawButton(2);
}
bool OI::GetRawButton4() {
	return driveJoystick->GetRawButton(2);
}

bool OI::GetToggleButton3() {
	if (!button3LastCalled && driveJoystick->GetRawButton(3)) {
		button3State = !button3State;
	}
	button3LastCalled = driveJoystick->GetRawButton(3);
	return button3State;
}
bool OI::GetToggleButton4() {
	if (!button4LastCalled && driveJoystick->GetRawButton(4)) {
		button4State = !button4State;
	}
	button4LastCalled = driveJoystick->GetRawButton(4);
	return button4State;
}
bool OI::GetSingularSecondaryButton2() {
	bool toReturn = (button2SecondaryLastCalled
			&& secondaryJoystick->GetRawButton(2));
	button2SecondaryLastCalled = secondaryJoystick->GetRawButton(2);
	return toReturn;
}

bool OI::GetSingularSecondaryButton3() {
	bool toReturn = (button3SecondaryLastCalled
			&& secondaryJoystick->GetRawButton(3));
	button2SecondaryLastCalled = secondaryJoystick->GetRawButton(3);
	return toReturn;
}
bool OI::GetSingularSecondaryButton4() {
	bool toReturn = (button4SecondaryLastCalled
			&& secondaryJoystick->GetRawButton(4));
	button4SecondaryLastCalled = secondaryJoystick->GetRawButton(4);
	return toReturn;
}
bool OI::GetSingularSecondaryButton5() {
	bool toReturn = (button5SecondaryLastCalled
			&& secondaryJoystick->GetRawButton(5));
	button5SecondaryLastCalled = secondaryJoystick->GetRawButton(5);
	return toReturn;
}
bool OI::GetSingularSecondaryButton6() {
	bool toReturn = (button6SecondaryLastCalled
			&& secondaryJoystick->GetRawButton(6));
	button6SecondaryLastCalled = secondaryJoystick->GetRawButton(6);
	return toReturn;
}
bool OI::GetSingularSecondaryButton7() {
	bool toReturn = (button7SecondaryLastCalled
			&& secondaryJoystick->GetRawButton(7));
	button6SecondaryLastCalled = secondaryJoystick->GetRawButton(7);
	return toReturn;
}
bool OI::GetSingularButton2() {
	bool toReturn = (button2LastCalled && driveJoystick->GetRawButton(2));
	button2LastCalled = driveJoystick->GetRawButton(2);
	return toReturn;
}

bool OI::GetSingularButton3() {
	bool toReturn = (button3LastCalled && driveJoystick->GetRawButton(3));
	button2LastCalled = driveJoystick->GetRawButton(3);
	return toReturn;
}
bool OI::GetSingularButton4() {
	bool toReturn = (button4LastCalled && driveJoystick->GetRawButton(4));
	button4LastCalled = driveJoystick->GetRawButton(4);
	return toReturn;
}
bool OI::GetSingularButton5() {
	bool toReturn = (button5LastCalled && driveJoystick->GetRawButton(5));
	button5LastCalled = driveJoystick->GetRawButton(5);
	return toReturn;
}
bool OI::GetSingularButton6() {
	bool toReturn = (button6LastCalled && driveJoystick->GetRawButton(6));
	button6LastCalled = driveJoystick->GetRawButton(6);
	return toReturn;
}
bool OI::GetTrigger() {
	if (!lastTriggered && driveJoystick->GetTrigger()) {
		triggerState = !triggerState;
	}
	lastTriggered = driveJoystick->GetTrigger();
	return triggerState;
}

double OI::GetThrottle() {
	return driveJoystick->GetThrottle() + 2.00000001;
}

Joystick* OI::GetJoystick() const {
	return driveJoystick;
}
