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
	double deadzone;
	double sensitivity;
	double bits = 12;

public:
	CorrectedGyro(AnalogInput* gyro, AnalogInput* temp);
	virtual ~CorrectedGyro();

	double GetVoltage();
	double GetAngle();
	double GetRate();
	double GetTemp();
	double GetRawValue();
	void Start();
	void Reset();
	void SetZeroVoltage(double zero);
	void SetDeadband(double deadzone);
	void SetSensitivity(double sensitivity);
};

#endif /* SRC_CORRECTEDGYRO_H_ */
