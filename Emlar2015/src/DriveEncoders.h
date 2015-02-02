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
	Encoder* frontLeftEncoder;
	Encoder* rearLeftEncoder;
	Encoder* frontRightEncoder;
	Encoder* rearRightEncoder;
public:
	DriveEncoders(Encoder* frontLeftEncoder, Encoder* rearLeftEncoder,
			Encoder* frontRightEncoder, Encoder* rearRightEncoder);
	double GetDistanceStraight();
	double GetDistanceStraif();
	double GetRotation();
	void ResetEncoders();
	virtual ~DriveEncoders();
};

#endif /* SRC_DRIVEENCODERS_H_ */
