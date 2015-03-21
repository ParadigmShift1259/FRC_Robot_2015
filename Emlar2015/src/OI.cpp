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
//	if (currentX > deadzone || currentX < -deadzone) {
	if (std::abs(currentX) > deadzone) {
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
double OI::GetSecondaryDeadZonedY() {
	double y = secondaryJoystick->GetY();
	//if (y < deadzone && y > -deadzone) {
	if(std::abs(y) < deadzone) {
		return 0.0;
	} else {
		return y;
	}
}
double OI::GetSecondaryRawY() {
	return secondaryJoystick->GetY();
}

double OI::GetY() {
	double currentY = (driveJoystick->GetY() / OI::GetThrottle());
	//adds deadzone and ramping
//	if (currentY > deadzone || currentY < -deadzone) {
	if (std::abs(currentY) > deadzone) {
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
//	if (currentTwist > twistDeadzone || currentTwist < -twistDeadzone) {
	if (std::abs(currentTwist) > twistDeadzone) {
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
	return driveJoystick->GetRawButton(3);
}
bool OI::GetRawButton4() {
	return driveJoystick->GetRawButton(4);
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
bool OI::GetSecondaryToggleButton2() {
	if (!secondaryButton2LastCalled && secondaryJoystick->GetRawButton(2)) {
		secondaryButton2State = !secondaryButton2State;
	}
	secondaryButton2LastCalled = secondaryJoystick->GetRawButton(2);
	return secondaryButton2State;
}
bool OI::GetSecondaryToggleButton3() {
	if (!secondaryButton3LastCalled && secondaryJoystick->GetRawButton(3)) {
		secondaryButton3State = !secondaryButton3State;
	}
	secondaryButton3LastCalled = secondaryJoystick->GetRawButton(3);
	return secondaryButton3State;
}
bool OI::GetSecondaryToggleButton4() {
	if (!secondaryButton4LastCalled && secondaryJoystick->GetRawButton(4)) {
		secondaryButton4State = !secondaryButton4State;
	}
	secondaryButton4LastCalled = secondaryJoystick->GetRawButton(4);
	return secondaryButton4State;
}
bool OI::GetSecondaryToggleButton5() {
	if (!secondaryButton5LastCalled && secondaryJoystick->GetRawButton(5)) {
		secondaryButton5State = !secondaryButton3State;
	}
	secondaryButton5LastCalled = secondaryJoystick->GetRawButton(5);
	return secondaryButton5State;
}

bool OI::GetSingularSecondaryButton2() {
	bool buttonState = secondaryJoystick->GetRawButton(2);
	bool toReturn = (!secondaryButton2LastCalled && buttonState);
	secondaryButton2LastCalled = buttonState;
	return toReturn;
}

bool OI::GetSingularSecondaryButton3() {
	bool buttonState = secondaryJoystick->GetRawButton(3);
	bool toReturn = (!secondaryButton3LastCalled && buttonState);
	secondaryButton3LastCalled = buttonState;
	return toReturn;
}

bool OI::GetSingularSecondaryButton4() {
	bool buttonState = secondaryJoystick->GetRawButton(4);
	bool toReturn = (!secondaryButton4LastCalled && buttonState);
	secondaryButton4LastCalled = buttonState;
	return toReturn;
}

bool OI::GetSingularSecondaryButton5() {
	bool buttonState = secondaryJoystick->GetRawButton(5);
	bool toReturn = (!secondaryButton5LastCalled && buttonState);
	secondaryButton5LastCalled = buttonState;
	return toReturn;
}

bool OI::GetSingularSecondaryButton6() {
	bool buttonState = secondaryJoystick->GetRawButton(6);
	bool toReturn = (!secondaryButton6LastCalled && buttonState);
	secondaryButton6LastCalled = buttonState;
	return toReturn;
}

bool OI::GetSingularSecondaryButton7() {
	bool buttonState = secondaryJoystick->GetRawButton(7);
	bool toReturn = (!secondaryButton7LastCalled && buttonState);
	secondaryButton7LastCalled = buttonState;
	return toReturn;
}

bool OI::GetSingularSecondaryButton8() {
	bool buttonState = secondaryJoystick->GetRawButton(8);
	bool toReturn = (!secondaryButton8LastCalled && buttonState);
	secondaryButton8LastCalled = buttonState;
	return toReturn;
}

bool OI::GetSingularSecondaryButton9() {
	bool buttonState = secondaryJoystick->GetRawButton(9);
	bool toReturn = (!secondaryButton9LastCalled && buttonState);
	secondaryButton9LastCalled = buttonState;
	return toReturn;
}

bool OI::GetSingularSecondaryButton10() {
	bool buttonState = secondaryJoystick->GetRawButton(10);
	bool toReturn = (!secondaryButton10LastCalled && buttonState);
	secondaryButton10LastCalled = buttonState;
	return toReturn;
}

bool OI::GetSingularSecondaryButton11() {
	bool buttonState = secondaryJoystick->GetRawButton(11);
	bool toReturn = (!secondaryButton11LastCalled && buttonState);
	secondaryButton11LastCalled = buttonState;
	return toReturn;
}
bool OI::GetToggleSecondaryButton11() {
	if (!secondaryButton11LastCalled && secondaryJoystick->GetRawButton(11)) {
		secondaryButton11TriggerState = !secondaryButton11TriggerState;
	}
	secondaryButton11LastCalled = secondaryJoystick->GetRawButton(11);
	return secondaryButton11TriggerState;
}

bool OI::GetSingularSecondaryButton12() {
	bool buttonState = secondaryJoystick->GetRawButton(12);
	bool toReturn = (!secondaryButton12LastCalled && buttonState);
	secondaryButton12LastCalled = buttonState;
	return toReturn;
}

bool OI::GetSingularButton2() {
	bool toReturn = (!button2LastCalled && driveJoystick->GetRawButton(2));
	button2LastCalled = driveJoystick->GetRawButton(2);
	return toReturn;
}

bool OI::GetSingularButton3() {
	bool toReturn = (!button3LastCalled && driveJoystick->GetRawButton(3));
	button2LastCalled = driveJoystick->GetRawButton(3);
	return toReturn;
}
bool OI::GetSingularButton4() {
	bool toReturn = (!button4LastCalled && driveJoystick->GetRawButton(4));
	button4LastCalled = driveJoystick->GetRawButton(4);
	return toReturn;
}
bool OI::GetSingularButton5() {
	bool toReturn = (!button5LastCalled && driveJoystick->GetRawButton(5));
	button5LastCalled = driveJoystick->GetRawButton(5);
	return toReturn;
}
bool OI::GetSingularButton6() {
	bool toReturn = (!button6LastCalled && driveJoystick->GetRawButton(6));
	button6LastCalled = driveJoystick->GetRawButton(6);
	return toReturn;
}
bool OI::GetSingularButton7() {
	bool toReturn = (!button7LastCalled && driveJoystick->GetRawButton(7));
	button7LastCalled = driveJoystick->GetRawButton(7);
	return toReturn;
}
bool OI::GetToggleTrigger() {
	if (!lastTriggered && driveJoystick->GetTrigger()) {
		triggerState = !triggerState;
	}
	lastTriggered = driveJoystick->GetTrigger();
	return triggerState;
}
bool OI::GetSecondaryToggleTrigger() {
	if (!secondaryLastTriggered && secondaryJoystick->GetTrigger()) {
		secondaryTriggerState = !secondaryTriggerState;
	}
	secondaryLastTriggered = secondaryJoystick->GetTrigger();
	return secondaryTriggerState;
}
bool OI::GetSingularSecondaryTrigger() {
	bool toReturn = (!secondaryLastTriggered && secondaryJoystick->GetTrigger());
	secondaryLastTriggered = secondaryJoystick->GetTrigger();
	return toReturn;
}

double OI::GetThrottle() {
	return driveJoystick->GetThrottle() + 2.0000000001;
}

Joystick* OI::GetJoystick() const {
	return driveJoystick;
}
