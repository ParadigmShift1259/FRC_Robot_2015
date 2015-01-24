/*
 * Thermister.cpp
 *
 *  Created on: Jan 24, 2015
 *      Author: paradigm
 */

#include <Thermister.h>

Thermister::Thermister(AnalogInput* thermister) {
	// TODO Auto-generated constructor stub
	this->thermister = thermister;
}

double Thermister::GetTemp() {
	double temp = thermister->GetVoltage();
	temp = ((temp-2.5)*9)+25;
	return temp;
}




Thermister::~Thermister() {
	// TODO Auto-generated destructor stub
}

