/*
 * IntakeWheels.h
 *
 *  Created on: Feb 14, 2015
 *      Author: paradigm
 */

#include "WPILib.h"

#ifndef SRC_INTAKEWHEELS_H_
#define SRC_INTAKEWHEELS_H_

class IntakeWheels {
private:
	SpeedController* wheels;
public:
	IntakeWheels(SpeedController* wheels);
	void EnableWheels();
	void DisableWheels();
	virtual ~IntakeWheels();
};

#endif /* SRC_INTAKEWHEELS_H_ */
