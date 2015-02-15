/*
 * Vacuum.cpp
 *
 *  Created on: Jan 29, 2015
 *      Author: paradigm
 */

#include <Vacuum.h>

Vacuum::Vacuum(SpeedController* vacuum) {
	this->vacuum = vacuum;
}

void Vacuum::Start() {
	vacuum->Set(0.42);
}

void Vacuum::Stop() {
	vacuum->Set(0.0);
}

Vacuum::~Vacuum() {
}
