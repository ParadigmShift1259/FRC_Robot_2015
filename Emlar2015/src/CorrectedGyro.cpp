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
	this->zero=zero;
	deadband = 4096.0*.014/5;
	zero = 4096.0/2.0;
	sensitivity = 0.007;
	this->sensitivity = sensitivity;
	gyro->SetOversampleBits(10);
	gyro->SetAverageBits(10);
	gyro->SetSampleRate(62500);
	gyro->SetAccumulatorCenter(zero);
	gyro->SetAccumulatorDeadband(deadband);
	gyro->InitAccumulator();
	Reset();
}

double CorrectedGyro::GetTemp() {
	double temperature = temp->GetVoltage();
	temperature = ((temperature-2.5)*9)+25;
	return temperature;
}

void CorrectedGyro::Reset() {
	int overSampleBits = gyro->GetOversampleBits();
	maxNum = 2^(overSampleBits);
	averageNumSize = 2^gyro->GetAverageBits();
	sampleRate = gyro->GetSampleRate();
	gyro->ResetAccumulator();
}

void CorrectedGyro::SetZeroValue(double value) {
	this->zero = value;
	gyro->SetAccumulatorCenter(value);
	Reset();
}

void CorrectedGyro::SetDeadband(double deadband) {
	this->deadband = deadband;
	gyro->SetAccumulatorDeadband(deadband);
	Reset();
}

void CorrectedGyro::SetSensitivity(double sensitivity) {
	this->sensitivity = sensitivity;
}

double CorrectedGyro::GetVoltage() {
	return gyro->GetVoltage();
}

double CorrectedGyro::GetAngle() {
	return gyro->GetAccumulatorValue()*5/(sensitivity*maxNum*sampleRate*averageNumSize);
}

double CorrectedGyro::GetRaw() {
	return gyro->GetValue();
}

double CorrectedGyro::GetRate() {
	return gyro->GetVoltage()/sensitivity;
}

CorrectedGyro::~CorrectedGyro() {

}

