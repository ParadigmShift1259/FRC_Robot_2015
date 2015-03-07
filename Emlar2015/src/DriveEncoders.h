/*
 * DriveEncoder.h
 *
 *  Created on: Jan 31, 2015
 *      Author: paradigm
 */

#ifndef SRC_DRIVEENCODERS_H_
#define SRC_DRIVEENCODERS_H_

#include "WPILib.h"

class DriveEncoders {
private:
	CANTalon* frontLeftTalon;
	CANTalon* backLeftTalon;
	CANTalon* frontRightTalon;
	CANTalon* backRightTalon;
public:
	DriveEncoders(CANTalon* frontRightTalon,
			CANTalon* backRightTalon, CANTalon* frontLeftTalon,
			CANTalon* backLeftTalon);
	double GetDistanceStraight();
	double GetDistanceStrafe();
	double GetRotation();
	void ResetEncoders();
	virtual ~DriveEncoders();
};

#endif /* SRC_DRIVEENCODERS_H_ */
