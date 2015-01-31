/*
 * Vacuum.cpp
 *
 *  Created on: Jan 29, 2015
 *      Author: paradigm
 */

#include <Vacuum.h>

Vacuum::Vacuum(AnalogInput* vacuumSensor, SpeedController* vacuum) {
	this->vacuumSensor = vacuumSensor;
	this->vacuum = vacuum;
}

void Vacuum::Start() {
	vacuum->Set(1.0);
}

void Vacuum::Stop() {
	vacuum->Set(0.0);
}



Vacuum::~Vacuum() {
}

