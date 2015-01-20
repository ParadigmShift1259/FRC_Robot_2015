#include <Joystick.h>
#include <OI.h>
#include <cmath>

/*

 * OI.cpp
 *
 *  Created on: Jan 16, 2015
 *      Author: Programming
 */

OI::OI(Joystick* mainJoystick2) {
	mainJoystick = mainJoystick2;
}
double OI::GetX() {
	double currentX = mainJoystick->GetX();
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
		x = 0;
	}
	return x;
}
double OI::GetY() {
	double currentY = mainJoystick->GetY();
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
		y = 0;
	}
	return y;
}
double OI::GetTwist() {
	double currentTwist = mainJoystick->GetTwist();
	//adds deadzone and ramping
	if (currentTwist > deadzone || currentTwist < -deadzone) {
		if (std::abs(currentTwist - twist) < shiftPerCycle) {
			twist = currentTwist;
		} else if (currentTwist > twist) {
			twist = twist + shiftPerCycle;
		} else {
			twist = twist - shiftPerCycle;
		}
	} else {
		twist = 0;
	}
	return twist;
}

bool OI::GetTrigger(){
	if(!lastTriggered&&mainJoystick->GetTrigger())
	{
		triggerState=!triggerState;
	}
	lastTriggered = mainJoystick->GetTrigger();
	return triggerState;
}
Joystick* OI::GetJoystick() const {
	return mainJoystick;
}

