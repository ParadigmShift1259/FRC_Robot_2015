#include "GyroPID.h"
#include "SmartDashboard/SmartDashboard.h"
#include "LiveWindow/LiveWindow.h"
#include "Gyro.h"

GyroPID::GyroPID(double p, double i, double d, Gyro* roboGyro, MechanumDriveTrain* driveTrain) :
		PIDSubsystem("GyroPID", p, i, d)
{
	this->driveTrain = driveTrain;
	this->roboGyro = roboGyro;
	GetPIDController()->SetAbsoluteTolerance(0.4);

	// Use these to get going:
	// SetSetpoint() -  Sets where the PID controller should move the system
	//                  to
	// Enable() - Enables the PID controller.
}

double GyroPID::ReturnPIDInput()
{
	return -roboGyro->GetAngle();
	// Return your input value for the PID loop
	// e.g. a sensor, like a potentiometer:
	// yourPot->SetAverageVoltage() / kYourMaxVoltage;	
}

void GyroPID::UsePIDOutput(double output)
{
	driveTrain->SetGyroPIDOffset(output);
	// Use output to drive your system, like a motor
	// e.g. yourMotor->Set(output);
}

void GyroPID::SetSetpoint(double setpoint) {
	GetPIDController()->SetSetpoint(setpoint);
}

void GyroPID::InitDefaultCommand()
{
	// Set the default command for a subsystem here.
	//setDefaultCommand(new MySpecialCommand());
}
