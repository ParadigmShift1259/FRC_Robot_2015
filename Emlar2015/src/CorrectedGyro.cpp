/*
 * CorrectedGyro.cpp
 *
 *  Created on: Jan 24, 2015
 *      Author: paradigm
 */

#include <CorrectedGyro.h>

/*
 * constructs the gyro using the analog channels for the gyro and teperature
 */
CorrectedGyro::CorrectedGyro(AnalogInput* gyro, AnalogInput* temp) {
	this->gyro = gyro;
	this->temp = temp;
	sensitivity = 0.007;
	gyro->SetSampleRate(65000);
	sampleRate = 65000;
	gyro->SetAccumulatorDeadband(deadband);
	gyro->SetAccumulatorInitialValue(0);
	sampleRate = gyro->GetSampleRate();
	double averageValue = (gyro->GetValue() + gyro->GetValue()
			+ gyro->GetValue() + gyro->GetValue()
			+ gyro->GetValue() + gyro->GetValue()
			+ gyro->GetValue() + gyro->GetValue()
			+ gyro->GetValue() + gyro->GetValue()
			+ gyro->GetValue() + gyro->GetValue()
			+ gyro->GetValue() + gyro->GetValue()
			+ gyro->GetValue() + gyro->GetValue()
			+ gyro->GetValue() + gyro->GetValue()
			+ gyro->GetValue() + gyro->GetValue()) / 20.0;
	gyro->SetAccumulatorCenter(averageValue);
	gyro->InitAccumulator();
	maxNum = 1 << 5;
}

double CorrectedGyro::GetTemp() {
	double temperature = temp->GetVoltage();
	temperature = ((temperature - 2.5) * 9.0) + 25.0;
	return temperature;
}

/**
 * resets the gyro by resetting the accumulator and resetting the zero value
 */
void CorrectedGyro::Reset() {
	sampleRate = gyro->GetSampleRate();
	double averageValue = (gyro->GetValue() + gyro->GetValue()
			+ gyro->GetValue() + gyro->GetValue()
			+ gyro->GetValue() + gyro->GetValue()
			+ gyro->GetValue() + gyro->GetValue()
			+ gyro->GetValue() + gyro->GetValue()
			+ gyro->GetValue() + gyro->GetValue()
			+ gyro->GetValue() + gyro->GetValue()
			+ gyro->GetValue() + gyro->GetValue()
			+ gyro->GetValue() + gyro->GetValue()
			+ gyro->GetValue() + gyro->GetValue()) / 20.0;
	gyro->SetAccumulatorCenter(averageValue);
	gyro->ResetAccumulator();
}

/**
 * sets the center value for the accumulator, again in raw ADC values
 */
void CorrectedGyro::SetZeroValue(double value) {
	this->zero = value;
	gyro->SetAccumulatorCenter(value);
	Reset();
}

/**
 * sets the minimum value to be interpreted by the accumulator
 * in raw ADC values
 */
void CorrectedGyro::SetDeadband(double deadband) {
	this->deadband = deadband;
	gyro->SetAccumulatorDeadband(deadband);
	Reset();
}

/**
 * sets the sensitivity eg. how many volts = 1 degree per second
 */
void CorrectedGyro::SetSensitivity(double sensitivity) {
	this->sensitivity = sensitivity;
}

/**
 * returns average voltage for whenever useful
 */
double CorrectedGyro::GetVoltage() {
	return gyro->GetAverageVoltage();
}

/**
 * converts raw accumulator output to an angle for easy readability
 */
double CorrectedGyro::GetAngle() {
	return gyro->GetAccumulatorValue() * 5.0
			/ (sensitivity * sampleRate * maxNum);
}

/**
 * returns the raw value of the
 */
double CorrectedGyro::GetRaw() {
	return gyro->GetValue();
}

/*
 * returns the rate the robot is turning in degrees per second
 */
double CorrectedGyro::GetRate() {
	return gyro->GetVoltage() / sensitivity;
}

CorrectedGyro::~CorrectedGyro() {

}
