/*
 * Lifter.cpp
 *
 *  Created on: Feb 2, 2015
 *      Author: paradigm
 */

#include <Lifter.h>

Lifter::Lifter(double p, double i, double d, CANTalon* lifterMotor,
		DoubleSolenoid* toteGrabber, DoubleSolenoid* toteDeployer,
		DoubleSolenoid* vacuumDeployer, Vacuum* vacuums, VacuumSensors* vacuumSensors, IntakeWheels* intakeWheels, int numberOfVacuums) {
	this->lifterMotor = lifterMotor;
	this->toteGrabber = toteGrabber;
	this->toteDeployer = toteDeployer;
	this->vacuumDeployer = vacuumDeployer;
	this->vacuums = vacuums;
	this->vacuumSensors = vacuumSensors;
	this->intakeWheels = intakeWheels;
	this->numberOfVacuums = numberOfVacuums;
	lifterMotor->SetPID(p, i, d);
}

bool Lifter::GrabbingTote() {
	return grabbingTote;
}

void Lifter::BeginAutoGrabTote() {
	if (!grabbingTote) {
		lifterMotor->Set(TOTE);
		currentSetpoint = TOTE;
		grabbingTote = true;
	}
}

void Lifter::Drop() {
	DeployTote();
	dropping = true;
	dropCount = 0;
}

bool Lifter::Zero() {
	bool lifterLowerLimitClosed = lifterMotor->IsRevLimitSwitchClosed();
	if(!lifterLowerLimitClosed){
		lifterMotor->SetControlMode(lifterMotor->kSpeed);
		lifterMotor->Set(-0.1);
	} else {
		lifterMotor->SetControlMode(lifterMotor->kPosition);
	}
	return lifterLowerLimitClosed;
}

void Lifter::ContinueDrop() {
	if(dropping){
		dropCount++;
	}
	if(dropCount/20.0 == 2) {
		ReleaseTote();
	}

}

void Lifter::RetractVacuum() {
	vacuumDeployer->Set(vacuumDeployer->kForward);
}
void Lifter::DeployTote() {
	toteDeployer->Set(toteDeployer->kReverse);
}
void Lifter::RetractTote() {
	toteDeployer->Set(toteDeployer->kForward);
}
void Lifter::GrabTote() {
	toteGrabber->Set(toteGrabber->kReverse);
}
void Lifter::ReleaseTote() {
	toteGrabber->Set(toteGrabber->kForward);
}
void Lifter::AutoGrabTote() {
	SmartDashboard::PutBoolean("Grabbing", grabbingTote);
	if (grabbingTote) {
		if (currentSetpoint == TOTE && InPos()) {
			StartVacuums();
			vacuumDeployer->Set(vacuumDeployer->kReverse);
		}
		if (currentSetpoint == FLOOR && InPos()) {
			GrabTote();
			grabbingTote = false;
		}
		if (VacuumsAttached()) {
			if (vacuumDeployer->Get() != vacuumDeployer->kForward && InPos()
					&& currentSetpoint == TOTE) {
				RetractVacuum();
				countSinceLastRetract = 0;
			}
			if (countSinceLastRetract == vacuumOffCount) {
				StopVacuums();
				ReleaseTote();
				lifterMotor->Set(FLOOR);
				currentSetpoint = FLOOR;
			}
			countSinceLastRetract++;
		}
	}
}
void Lifter::StartVacuums() {
	for (int i = 0; i<numberOfVacuums; i++) {
		vacuums[i].Start();
	}
}
void Lifter::StopVacuums() {
	for (int i = 0; i<numberOfVacuums; i++) {
		vacuums[i].Start();
	}
}
bool Lifter::VacuumsAttached() {
	return vacuumSensors->IsAttached();
}

bool Lifter::InPos() {
	double error = currentSetpoint - lifterMotor->GetPosition();
	return (error < threshold && error > -threshold);
}

void Lifter::MoveTo(double setpoint) {
	lifterMotor->Set(setpoint);
	currentSetpoint = setpoint;
}

Lifter::~Lifter() {
	// TODO Auto-generated destructor stub
}
