/*
 * Lifter.cpp
 *
 *  Created on: Feb 2, 2015
 *      Author: paradigm
 */

#include <Lifter.h>
#include <math.h>

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
	//Note that the first argument is actually the function handle, not the return value of the function
	this->vacuumRetractNotifier = new Notifier(
			Lifter::VacuumRetractNotifierHandler, this);
	this->toteGrabberRetractNotifier = new Notifier(
			Lifter::ToteGrabberRetractNotifierHandler, this);
	this->toteGrabberExtendNotifier = new Notifier(
			Lifter::ToteGrabberExtendNotifierHandler, this);
	this->settleNotifier = new Notifier(Lifter::SettleNotifierHandler, this);

}

bool Lifter::GrabbingTote() {
	return grabbingTote;
}

bool Lifter::DroppingTote() {
	return dropping;
}

void Lifter::BeginAutoGrabTote() {
	if (!grabbingTote) {
		lifterMotor->Set(VACUUMCLEARANCE);
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

bool Lifter::Cleared() {
	return cleared;
}

void Lifter::Clear() {
	clearing = true;
	cleared = false;
	DeployTote();
	countSinceArmDeployTriggered = 0;
}

bool Lifter::JustStarted() {
	return justStarted;
}

void Lifter::EndStartPeriod() {
	justStarted = false;
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
	if (clearing) {
		countSinceArmDeployTriggered++;
		if(countSinceArmDeployTriggered == ARM_DEPLOY_TIME ) {
			RetractTote();
			clearing = false;
			cleared = true;
		}
	}
	//tote dropping
	if (dropping) {
		dropCount++;
		if (dropCount > ARM_DEPLOY_TIME && InPos() && lowered) {
			lowered = true;
			ReleaseTote();
		}
		if (dropCount == ARM_DEPLOY_TIME) {
			lowered = true;
			lifterMotor->Set(lifterMotor->GetSetpoint() - DROPDISTANCE);
		}
		if(lowered == true && InPos()){
			RetractTote();
			lifterMotor->Set(VACUUMCLEARANCE);
		}
		if(toteDeployer->Get() == toteDeployer->kReverse) {
			dropping = false;
			lowered = false;
		}
	}
	//tote grabbing
	if (grabbingTote) {
		if ((countSinceToteGrabberRetractTriggered > GRABBER_RETRACT_TIME)
				&& (countSinceVacuumRetractTriggered > VACUUM_RETRACT_TIME)
				&& (countSinceToteGrabberExtendTriggered > GRABBER_EXTEND_TIME)
				&& (countSinceSettleTriggered > SETTLE_TIME)) {
			if (((vacuumDeployer->Get() == vacuumDeployer->kReverse)
					|| (vacuumDeployer->Get() == vacuumDeployer->kOff))
					&& SafeChangeVacuumState()
					&& lifterMotor->GetSetpoint() == VACUUMCLEARANCE
					&& !VacuumsAttached()) {
				printf("deploying vacuum\n");
				StartVacuums();
				intakeWheels->EnableWheels();
				DeployVacuum();
				SmartDashboard::PutBoolean("Deploying Vacuum", true);
			}
			if ((vacuumDeployer->Get() == vacuumDeployer->kForward)
					&& (VacuumsAttached() || skipVacuumSensors) && InPos()
					&& (lifterMotor->GetSetpoint() == VACUUMCLEARANCE)) {
				RetractVacuum();
				SmartDashboard::PutBoolean("Deploying Vacuum", false);
				skipVacuumSensors = false;
				countSinceVacuumRetractTriggered = 0;
				if (useNotifier) {
					vacuumRetractNotifier->StartSingle(
							VACUUM_RETRACT_TIME / 20.0);
				}
			}
			if ((lifterMotor->GetSetpoint() == (VACUUMCLEARANCE - DROPDISTANCE))
					&& InPos()) {
				ReleaseTote();
				countSinceToteGrabberExtendTriggered = 0;
				if (useNotifier) {
					toteGrabberExtendNotifier->StartSingle(
							GRABBER_EXTEND_TIME / 20.0);
				}
				intakeWheels->DisableWheels();
			}

			if ((lifterMotor->GetSetpoint() == FLOOR) && InPos()) {
				countSinceSettleTriggered = 0;
				if (useNotifier) {
					settleNotifier->StartSingle(SETTLE_TIME / 20.0);
				}
			}
		}
		if (countSinceVacuumRetractTriggered == VACUUM_RETRACT_TIME) {
			printf("releasing tote on other tote\n");
			StopVacuums();
			lifterMotor->Set(VACUUMCLEARANCE - DROPDISTANCE);
			if (useNotifier) {
				countSinceVacuumRetractTriggered++;
			}
		}
		if ((countSinceSettleTriggered == SETTLE_TIME)) {
			GrabTote();
			countSinceToteGrabberRetractTriggered = 0;
			if (useNotifier) {
				toteGrabberRetractNotifier->StartSingle(
						GRABBER_RETRACT_TIME / 20.0);
				countSinceSettleTriggered++;
			}
		}
		if ((countSinceToteGrabberRetractTriggered == GRABBER_RETRACT_TIME)) {
			printf("ToteGrabbed\n");
			lifterMotor->Set(VACUUMCLEARANCE);
			grabbingTote = false;
			if (useNotifier) {
				countSinceToteGrabberRetractTriggered++;
			}
		}
		if (countSinceToteGrabberExtendTriggered == GRABBER_EXTEND_TIME) {
			lifterMotor->Set(FLOOR);
			if (useNotifier) {
				countSinceToteGrabberExtendTriggered++;
			}
		}
		if (!useNotifier) {
			countSinceToteGrabberRetractTriggered++;
			countSinceVacuumRetractTriggered++;
			countSinceToteGrabberExtendTriggered++;
			countSinceSettleTriggered++;
		}
	}
	if (emergencyClearing && InPos()) {
		emerClearCount++;
		dropping = false;
		grabbingTote = false;
	}
	if (emerClearCount / 50 == 1) {
		emergencyClearing = false;
		emerClearCount = 0;
	}

}
void Lifter::Disable() {
	grabbingTote = false;
	dropping = false;
	emergencyClearing = false;
	zeroed = false;
	lowered = false;
	countSinceVacuumRetractTriggered = VACUUM_RETRACT_TIME + 1;
	vacuumRetractNotifier->Stop();
	countSinceToteGrabberRetractTriggered = GRABBER_RETRACT_TIME + 1;
	toteGrabberRetractNotifier->Stop();
	StopVacuums();
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

bool Lifter::Clearing() {
	return clearing;
}
void Lifter::DeployTote() {
	if ((lifterMotor->GetPosition() > ((VACUUMCLEARANCE - THRESHOLD) / 4))||!cleared) {
		toteDeployer->Set(toteDeployer->kReverse);
	}
}
void Lifter::RetractTote() {
	if ((lifterMotor->GetPosition() > ((VACUUMCLEARANCE - THRESHOLD) / 4))||!cleared) {
		toteDeployer->Set(toteDeployer->kForward);
	}
}
void Lifter::GrabTote() {
	if (vacuumDeployer->Get() != vacuumDeployer->kForward) {
		toteGrabber->Set(toteGrabber->kReverse);
	}
}
void Lifter::ReleaseTote() {
	if (vacuumDeployer->Get() != vacuumDeployer->kForward) {
		toteGrabber->Set(toteGrabber->kForward);
	}
}

void Lifter::SkipVacuumSensors() {
	skipVacuumSensors = !skipVacuumSensors;
}

void Lifter::StartVacuums() {
	for (int i = 0; i < numberOfVacuums; i++) {
		vacuums[i]->Start();
	}
}
void Lifter::StopVacuums() {
	for (int i = 0; i < numberOfVacuums; i++) {
		vacuums[i]->Stop();
	}
}
bool Lifter::VacuumsAttached() {
	return vacuumSensors->IsAttached();
}

bool Lifter::SafeChangeVacuumState() {
	return ((-lifterMotor->GetEncPosition())
			> ((VACUUMCLEARANCE - THRESHOLD) / 2.0));
}

bool Lifter::InPos() {
	double error = lifterMotor->GetClosedLoopError();
	return std::abs(error) < THRESHOLD;
}

void Lifter::StartEmergencyClear() {
	emergencyClearing = true;
	grabbingTote = false;
	dropping = false;
	lifterMotor->Set(VACUUMCLEARANCE);
	StopVacuums();
	DeployVacuum();
}

void Lifter::MoveTo(double setpoint) {
	if (!(grabbingTote || dropping)) {
		lifterMotor->Set(setpoint);
	}
}

void Lifter::VacuumRetractNotifierHandler(void* lifter) {
	Lifter* trigger = (Lifter*) lifter;
	trigger->TriggerVacuumRetract();
}

void Lifter::ToteGrabberRetractNotifierHandler(void* lifter) {
	Lifter* trigger = (Lifter*) lifter;
	trigger->TriggerToteGrabberRetract();
}

void Lifter::ToteGrabberExtendNotifierHandler(void* lifter) {
	Lifter* trigger = (Lifter*) lifter;
	trigger->TriggerToteGrabberExtend();
}

void Lifter::SettleNotifierHandler(void* lifter) {
	Lifter* trigger = (Lifter*) lifter;
	trigger->TriggerSettle();
}

void Lifter::TriggerVacuumRetract() {
	countSinceVacuumRetractTriggered = VACUUM_RETRACT_TIME;
}

void Lifter::TriggerToteGrabberRetract() {
	countSinceToteGrabberRetractTriggered = GRABBER_RETRACT_TIME;
}

void Lifter::TriggerToteGrabberExtend() {
	countSinceToteGrabberExtendTriggered = GRABBER_EXTEND_TIME;
}

void Lifter::TriggerSettle() {
	countSinceSettleTriggered = SETTLE_TIME;
}

Lifter::~Lifter() {
// TODO Auto-generated destructor stub
	delete vacuumRetractNotifier;
	delete toteGrabberRetractNotifier;
	delete toteGrabberExtendNotifier;
	delete settleNotifier;
}
