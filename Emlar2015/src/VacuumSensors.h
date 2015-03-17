/*
 * VacuumSensors.h
 *
 *  Created on: Feb 13, 2015
 *      Author: paradigm
 */
#include "WPILib.h"


#ifndef SRC_VACUUMSENSORS_H_
#define SRC_VACUUMSENSORS_H_

class VacuumSensors {
private:
	unsigned int threshold;
	SPI* vacuumSpi;
	uint8_t receive[4];
	uint8_t send[4];
public:
	VacuumSensors(SPI* vacuumSpi);
	bool IsAttached();
	unsigned int GetCH0();
	unsigned int GetCH1();
	unsigned int GetCH2();
	unsigned int GetCH3();
	unsigned int GetCH4();
	unsigned int GetCH5();
	unsigned int GetCH6();
	unsigned int GetCH7();
	bool CH0Attached();
	bool CH1Attached();
	bool CH2Attached();
	bool CH3Attached();
	bool CH4Attached();
	bool CH5Attached();
	bool CH6Attached();
	bool CH7Attached();
	double GetCH0Voltage();
	double GetCH1Voltage();
	double GetCH2Voltage();
	double GetCH3Voltage();
	double GetCH4Voltage();
	double GetCH5Voltage();
	double GetCH6Voltage();
	double GetCH7Voltage();
	virtual ~VacuumSensors();

};

#endif /* SRC_VACUUMSENSORS_H_ */
