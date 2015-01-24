/*
 * Thermister.h
 *
 *  Created on: Jan 24, 2015
 *      Author: paradigm
 */

#ifndef SRC_THERMISTER_H_
#define SRC_THERMISTER_H_

#include "WPILib.h"

class Thermister {
private:
	AnalogInput* thermister;

public:
	Thermister(AnalogInput* thermister);
	virtual ~Thermister();
	double GetTemp();
};

#endif /* SRC_THERMISTER_H_ */
