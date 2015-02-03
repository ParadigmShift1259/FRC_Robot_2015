/*
 * Lifter.cpp
 *
 *  Created on: Feb 2, 2015
 *      Author: paradigm
 */

#include <Lifter.h>

Lifter::Lifter(double p, double i, double d, CANTalon* lifterMotor) {
	this->lifterMotor = lifterMotor;
	lifterMotor->SetPID(p, i, d);
}

bool Lifter::InPos() {
	double error = levels[currentSetpoint] - lifterMotor->GetPosition();
	return (error < threshold && error > -threshold);
}

void Lifter::MoveTo(int step) {
	if (step <= 7 && step >= 0) {
		lifterMotor->SetPosition(levels[step]);
	}
}

Lifter::~Lifter() {
	// TODO Auto-generated destructor stub
}

