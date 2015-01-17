#include <Joystick.h>
#include <OI.h>

/*

* OI.cpp
 *
 *  Created on: Jan 16, 2015
 *      Author: Programming
 */


OI::OI(Joystick& mainJoystick2){
	mainJoystick = &mainJoystick2;
}
void OI::deadD(){
	if (mainJoystick->GetX() > deadzone || mainJoystick->GetX() < -deadzone){
		x = mainJoystick->GetX();
	} else {
		x = 0;
	}
	if (mainJoystick->GetY() > deadzone || mainJoystick->GetY() < -deadzone){
		y = mainJoystick->GetY();
	} else {
		y = 0;
	}

	if (mainJoystick->GetTwist() > deadzone || mainJoystick->GetTwist() < -deadzone){
		twist = mainJoystick->GetTwist();
	} else {
		twist = 0;
	}
}
double OI::getX() const{
	return x;
}
double OI::getY() const{
	return y;
}
double OI::getTwist() const{
	return twist;
}

Joystick* OI::getJoystick() const{
	return mainJoystick;
}




