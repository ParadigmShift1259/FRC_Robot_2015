/*
 * CorrectedGyro.h
 *
 *  Created on: Jan 24, 2015
 *      Author: paradigm
 */

#ifndef SRC_CORRECTEDGYRO_H_
#define SRC_CORRECTEDGYRO_H_

#include "WPILib.h"

class CorrectedGyro {
private:
	AnalogInput* gyro;
	AnalogInput* temp;
	double zero;
	double deadband;
	double sensitivity;
	double maxNum;
	double averageNumSize;
	double sampleRate;


public:
	CorrectedGyro(AnalogInput* gyro, AnalogInput* temp);
	virtual ~CorrectedGyro();

	double GetVoltage();
	double GetAngle();
	double GetRate();
	double GetTemp();
	double GetRaw();

	void Reset();
	void SetZeroValue(double value);
	void SetDeadband(double deadzone);
	void SetSensitivity(double sensitivity);
};

#endif /* SRC_CORRECTEDGYRO_H_ */
