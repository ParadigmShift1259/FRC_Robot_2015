/*
 * IntakeWheels.cpp
 *
 *  Created on: Feb 14, 2015
 *      Author: paradigm
 */

#include <IntakeWheels.h>

IntakeWheels::IntakeWheels(SpeedController* wheels) {
	this->wheels = wheels;
}

void IntakeWheels::EnableWheels() {
	wheels->Set(1.0);
}

void IntakeWheels::DisableWheels() {
	wheels->Set(0.0);
}

IntakeWheels::~IntakeWheels() {
	// TODO Auto-generated destructor stub
}

