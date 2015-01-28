/*
 * CorrectedGyro.cpp
 *
 *  Created on: Jan 24, 2015
 *      Author: paradigm
 */

#include <CorrectedGyro.h>

CorrectedGyro::CorrectedGyro(AnalogInput* gyro, AnalogInput* temp) {
	this->gyro = gyro;
	this->temp = temp;
	this->zero=  gyro->GetValue();
	this->deadzone = 3;
	this->sensitivity = 0.007;
	gyro->SetAccumulatorInitialValue(0.0);
}

void CorrectedGyro::Start() {
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
	//this->zero = zero*819.2;
	gyro->SetAccumulatorCenter(zero);
}

void CorrectedGyro::SetDeadband(double deadzone) {
	this->deadzone = deadzone;
	gyro->SetAccumulatorDeadband(deadzone);
}

void CorrectedGyro::SetSensitivity(double sensitivity) {
	this->sensitivity = sensitivity;
}

double CorrectedGyro::GetVoltage() {
	return gyro->GetVoltage();
}

double CorrectedGyro::GetRawValue() {
	return gyro->GetValue();
}

double CorrectedGyro::GetAngle() {
	double value;
	value = gyro->GetAccumulatorValue();
	double volts = ((gyro->GetLSBWeight() * 10^-9) * value) - (gyro->GetOffset * 10^-9);
	return volts/sensitivity;
}

double CorrectedGyro::GetRate() {
	return gyro->GetVoltage()/sensitivity;
}

CorrectedGyro::~CorrectedGyro() {

}

