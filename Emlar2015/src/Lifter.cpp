/*
 * Lifter.cpp
 *
 *  Created on: Feb 2, 2015
 *      Author: paradigm
 */

#include <Lifter.h>

Lifter::Lifter(double p, double i, double d, CANTalon* lifterMotor,
		DoubleSolenoid* toteGrabber, DoubleSolenoid* toteDeployer,
		DoubleSolenoid* vacuumDeployer, Vacuum* vacuum1) {
	this->lifterMotor = lifterMotor;
	this->toteGrabber = toteGrabber;
	this->toteDeployer = toteDeployer;
	this->vacuumDeployer = vacuumDeployer;
	this->vacuum1 = vacuum1;

	lifterMotor->SetPID(p, i, d);
}

void Lifter::BeginAutoGrabTote() {
	if (!grabbingTote) {
		lifterMotor->SetPosition(levels[TOTE]);
		currentSetpoint = TOTE;
		grabbingTote = true;
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
				lifterMotor->SetPosition(levels[FLOOR]);
				currentSetpoint = FLOOR;
			}
			countSinceLastRetract++;
		}
	}
}
void Lifter::StartVacuums() {
	vacuum1->Start();
}
void Lifter::StopVacuums() {
	vacuum1->Stop();
}
bool Lifter::VacuumsAttached() {
	return vacuum1->IsAttached();
}

bool Lifter::InPos() {
	double error = levels[currentSetpoint] - lifterMotor->GetPosition();
	return (error < threshold && error > -threshold);
}

void Lifter::MoveTo(int step) {
	if (step <= 7 && step >= 0 && !grabbingTote) {
		lifterMotor->SetPosition(levels[step]);
	}
}

Lifter::~Lifter() {
	// TODO Auto-generated destructor stub
}

