/*
 * AccelToPos.h
 *
 *  Created on: Jan 23, 2015
 *      Author: Programming
 */

#ifndef SRC_ACCELTODISP_H_
#define SRC_ACCELTODISP_H_

#include "Wpilib.h"

class AccelToDisp {
private:
	const double timePerCall = 0.02; //in seconds for niceness
	const double gForceToMetric = 9.80665; //coefficient to convert g-force to m/s^2
	const double accelConversionCoeeficient = timePerCall*gForceToMetric/2; //used to minimize math
	const double halfedTimePerCall = timePerCall/2;

	double lastxAccel = 0;
	double lastyAccel = 0;
	double lastzAccel = 0;

	double xVelocity = 0;
	double yVelocity = 0;
	double zVelocity = 0;

	double lastxVelocity = 0;
	double lastyVelocity = 0;
	double lastzVelocity = 0;

	double xDisp = 0;
	double yDisp = 0;
	double zDisp = 0;

	double lastxDisp = 0;
	double lastyDisp = 0;
	double lastzDisp = 0;

	Accelerometer* accel;

	void CalcVelocities();

public:
	AccelToDisp(Accelerometer* accel);
	virtual ~AccelToDisp();
	double GetXVelocity() const;
	double GetYVelocity() const;
	double GetZVelocity() const;
	double GetXDisp() const;
	double GetYDisp() const;
	double GetZDisp() const;
	void CalcDisplacements();
};

#endif /* SRC_ACCELTODISP_H_ */
