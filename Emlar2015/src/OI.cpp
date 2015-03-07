#include <Joystick.h>
#include <OI.h>
#include <cmath>

/*

 * OI.cpp
 *
 *  Created on: Jan 16, 2015
 *      Author: Programming
 */

OI::OI(Joystick* mainJoystick) {
	this->mainJoystick = mainJoystick;

}
double OI::GetX() {
	double currentX = (mainJoystick->GetX() / OI::GetThrottle());
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
	return mainJoystick->GetY();
}

double OI::GetY() {
	double currentY = (mainJoystick->GetY() / OI::GetThrottle());
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
	double currentTwist = (mainJoystick->GetTwist() / OI::GetThrottle());
	//adds deadzone
	if (currentTwist > twistDeadzone || currentTwist < -twistDeadzone) {
		twist = currentTwist;
	} else {
		twist = 0;
	}
	return twist;
}

bool OI::GetButton2() {
	if (!button2LastCalled && mainJoystick->GetRawButton(2)) {
		button2State = !button2State;
	}
	button2LastCalled = mainJoystick->GetRawButton(2);
	return button2State;
}

bool OI::GetButton3() {
	if (!button3LastCalled && mainJoystick->GetRawButton(3)) {
		button3State = !button3State;
	}
	button3LastCalled = mainJoystick->GetRawButton(3);
	return button3State;
}
bool OI::GetButton4() {
	if (!button4LastCalled && mainJoystick->GetRawButton(4)) {
		button4State = !button4State;
	}
	button4LastCalled = mainJoystick->GetRawButton(4);
	return button4State;
}

bool OI::GetTrigger() {
	if (!lastTriggered && mainJoystick->GetTrigger()) {
		triggerState = !triggerState;
	}
	lastTriggered = mainJoystick->GetTrigger();
	return triggerState;
}

double OI::GetThrottle() {
	return mainJoystick->GetThrottle() + 2.00000001;
}

Joystick* OI::GetJoystick() const {
	return mainJoystick;
}
