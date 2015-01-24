/*
 * CorrectedGyro.cpp
 *
 *  Created on: Jan 24, 2015
 *      Author: paradigm
 */

#include <CorrectedGyro.h>

CorrectedGyro::CorrectedGyro(AnalogInput* gyro, AnalogInput* temp ,double zero, double deadzone, double sensitivity) {
	this->gyro = gyro;
	this->temp = temp;
	this->zero=zero;
	this->deadzone = deadzone;
	this->sensitivity = sensitivity;
	gyro->SetAccumulatorCenter(zero);
	gyro->SetAccumulatorDeadband(deadzone);
	gyro->InitAccumulator();
}

double CorrectedGyro::GetTemp() {
	double temperature = temp->GetVoltage();
	temperature = ((temperature-2.5)*9)+25;
	return temperature;
}

void CorrectedGyro::Reset() {
	gyro->ResetAccumulator();
}

void CorrectedGyro::SetZeroVoltage(double zero) {
	this->zero = zero;
}

void CorrectedGyro::SetDeadzone(double deadzone) {
	this->deadzone = deadzone;
}

void CorrectedGyro::SetSensitivity(double sensitivity) {
	this->sensitivity = sensitivity;
}

double CorrectedGyro::GetVoltage() {
	return gyro->GetVoltage();
}

double CorrectedGyro::GetAngle() {
	return gyro->GetAccumulatorValue()/sensitivity;
}

double CorrectedGyro::GetRate() {
	return gyro->GetVoltage()/sensitivity;
}

CorrectedGyro::~CorrectedGyro() {

}

