/*
 * Lifter.cpp
 *
 *  Created on: Feb 2, 2015
 *      Author: paradigm
 */

#include <Lifter.h>

Lifter::Lifter(double p, double i, double d, CANTalon* lifterMotor,
		DoubleSolenoid* toteGrabber, DoubleSolenoid* toteDeployer,
		DoubleSolenoid* vacuumDeployer, Vacuum** vacuums,
		VacuumSensors* vacuumSensors, IntakeWheels* intakeWheels,
		int numberOfVacuums) {
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
		lifterMotor->Set(VACUUMCLEARANCE);
		currentSetpoint = VACUUMCLEARANCE;
		grabbingTote = true;
	}
}

void Lifter::Drop() {
	DeployTote();
	dropping = true;
	dropCount = 0;
}

bool Lifter::Zeroed() {
	return zeroed;
}

void Lifter::Zero() {
	bool lifterLowerLimitClosed = lifterMotor->IsRevLimitSwitchClosed();
	if (!lifterLowerLimitClosed) {
		lifterMotor->SetControlMode(lifterMotor->kPercentVbus);
		lifterMotor->Set(-0.3);
		zeroed = false;
	} else {
		lifterMotor->SetPosition(0.0);
		lifterMotor->SetControlMode(lifterMotor->kPosition);
		zeroed = true;
	}
}

void Lifter::LifterQueuedFunctions() {
	//tote dropping
	/*
	 if (dropping) {
	 dropCount++;
	 }
	 if (dropCount / 20.0 == 2) {
	 ReleaseTote();
	 }
	 */
	//tote grabbing
	if (grabbingTote) {
		if ((grabWait > grabOffCount) && (settleWait > settleOffCount)
				&& (countSinceLastRetract > vacuumOffCount)) {
			if (((vacuumDeployer->Get() == vacuumDeployer->kReverse)
					|| (vacuumDeployer->Get() == vacuumDeployer->kOff))
					&& SafeChangeVacuumState()) {
				printf("deploying vacuum\n");
				StartVacuums();
				DeployVacuum();
				SmartDashboard::PutBoolean("Deploying Vacuum", true);
			}
			if (currentSetpoint == FLOOR && InPos()) {
				printf("grabbedTote\n");
				GrabTote();
				settleWait = 0;
				lifterMotor->Set(VACUUMCLEARANCE);
				currentSetpoint = VACUUMCLEARANCE;
				grabbingTote = false;
			}
			if ((vacuumDeployer->Get() == vacuumDeployer->kForward)
					&& VacuumsAttached()) {
				if (InPos() && currentSetpoint == VACUUMCLEARANCE) {
					RetractVacuum();
					countSinceLastRetract = 0;
				}
			}
		}
		if ((countSinceLastRetract == vacuumOffCount) && (settleWait > settleOffCount) && (grabWait > grabOffCount)) {
			printf("releasing tote on other tote\n");
			StopVacuums();
			ReleaseTote();
			lifterMotor->Set(FLOOR);
			currentSetpoint = FLOOR;
		}
		if ((settleWait == settleOffCount) && InPos()
				&& (countSinceLastRetract > vacuumOffCount) && (grabWait > grabOffCount)) {
			GrabTote();
			grabWait = 0;
		}
		if ((grabWait == grabOffCount) && (settleWait > settleOffCount)
				&& (countSinceLastRetract > vacuumOffCount)) {
			lifterMotor->Set(VACUUMCLEARANCE);
		}
		settleWait++;
		grabWait++;
		countSinceLastRetract++;
	}
	//emergency clear
	/*
	 if(emergencyClearing) {
	 clearCount++;
	 }
	 if(clearCount/20 == 1) {
	 emergencyClearing = false;
	 clearCount = 0;
	 }
	 */

}
void Lifter::Disable() {
	grabbingTote = false;
	emergencyClearing = false;
}

void Lifter::DeployVacuum() {
	if (SafeChangeVacuumState()) {
		vacuumDeployer->Set(vacuumDeployer->kForward);
	}
}
void Lifter::RetractVacuum() {
	if (SafeChangeVacuumState()) {
		vacuumDeployer->Set(vacuumDeployer->kReverse);
	}
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

void Lifter::StartVacuums() {
	vacuums[0]->Start();
	vacuums[1]->Start();
	vacuums[2]->Start();
	vacuums[3]->Start();
	vacuums[4]->Start();
}
void Lifter::StopVacuums() {
	vacuums[0]->Stop();
	vacuums[1]->Stop();
	vacuums[2]->Stop();
	vacuums[3]->Stop();
	vacuums[4]->Stop();
}
bool Lifter::VacuumsAttached() {
	return vacuumSensors->IsAttached();
}

bool Lifter::SafeChangeVacuumState() {
	return ((-lifterMotor->GetEncPosition()) > (VACUUMCLEARANCE - threshold));
}

bool Lifter::InPos() {
	double error = lifterMotor->GetClosedLoopError();
	return (error < threshold && error > -threshold);
}

void Lifter::StartEmergencyClear() {
	emergencyClearing = true;
	grabbingTote = false;
	StopVacuums();
	DeployVacuum();
}

void Lifter::MoveTo(double setpoint) {
	if (!grabbingTote) {
		lifterMotor->Set(setpoint);
		currentSetpoint = setpoint;
	}
}

Lifter::~Lifter() {
// TODO Auto-generated destructor stub
}
