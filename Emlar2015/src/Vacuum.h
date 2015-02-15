/*
 * Vacuum.h
 *
 *  Created on: Jan 29, 2015
 *      Author: paradigm
 */

#ifndef SRC_VACUUM_H_
#define SRC_VACUUM_H_

#include "WPILib.h"

class Vacuum {
	SpeedController* vacuum;
public:
	Vacuum(SpeedController* vacuum);
	void Start();
	void Stop();
	bool IsAttached();
	virtual ~Vacuum();
};

#endif /* SRC_VACUUM_H_ */
