#include "WPIlib.h"

/*

* OI.cpp
 *
 *  Created on: Jan 16, 2015
 *      Author: Programming
 */
class OI {
private:
	Joystick leftJoystick;
public:
	OI(uint32_t leftJoystickChannel):
		leftJoystick(leftJoystickChannel) {

	}

	double GetX() {
		return leftJoystick.GetX();
	}

	double GetY() {
		return leftJoystick.GetY();
	}

	double GetZ() {
		return leftJoystick.GetZ();
	}
};



