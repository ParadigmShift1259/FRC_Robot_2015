/*
 * AccelToPos.cpp
 *
 *  Created on: Jan 23, 2015
 *      Author: Programming
 */

#include <AccelToDisp.h>
#include "WPILib.h"

AccelToDisp::AccelToDisp(Accelerometer* accel) {
	this->accel = accel;
}

double AccelToDisp::GetXVelocity() const {
	return xVelocity;
}

double AccelToDisp::GetYVelocity() const {
	return yVelocity;
}

double AccelToDisp::GetZVelocity() const {
	return zVelocity;
}

void AccelToDisp::CalcVelocities() { //dat calculus

	//gets the data from the accelerometers and converts it to m/s^2 and then m/s
	//this is also divided by two to allow for easy averaging later
	double currentxAccel = accel->GetX()*accelConversionCoeeficient;
	double currentyAccel = accel->GetY()*accelConversionCoeeficient;
	double currentzAccel = accel->GetZ()*accelConversionCoeeficient;

	//adds the average velocity over the past period
	xVelocity = xVelocity + (currentxAccel+lastxAccel);
	yVelocity = yVelocity + (currentyAccel+lastyAccel);
	zVelocity = zVelocity + (currentzAccel+lastzAccel);

	//sets the last acceleration to the current acceleration to prep for the next call
	lastxAccel = currentxAccel;
	lastyAccel = currentyAccel;
	lastzAccel = currentzAccel;
}

void AccelToDisp::CalcDisplacements() { //moar calculus
	CalcVelocities(); //first calculates the current velocity so this calculation can be done

	//averages the velocity and multiplies it by the call time to get displacement
	xDisp = xDisp + (xVelocity+lastxVelocity)*halfedTimePerCall;
	yDisp = yDisp + (yVelocity+lastyVelocity)*halfedTimePerCall;
	zDisp = zDisp + (zVelocity+lastzVelocity)*halfedTimePerCall;

	//sets the last velocity to the current velocity to prep for the next call
	lastxVelocity = xVelocity;
	lastyVelocity = yVelocity;
	lastzVelocity = zVelocity;
}

AccelToDisp::~AccelToDisp() {
	// TODO Auto-generated destructor stub
}

