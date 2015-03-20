/*
 * VacuumSensors.cpp
 *
 *  Created on: Feb 13, 2015
 *      Author: paradigm
 */

#include <VacuumSensors.h>

VacuumSensors::VacuumSensors(SPI* vacuumSpi) {
	this->vacuumSpi = vacuumSpi;
	threshold = 850;
}

bool VacuumSensors::IsAttached() {
	bool CH0Attach = CH0Attached();
	bool CH1Attach = CH1Attached();
	bool CH2Attach = CH2Attached();
	bool CH3Attach = CH3Attached();
	bool CH4Attach = CH4Attached();
	int numberAttached =  (int)CH0Attach + (int)CH1Attach + (int)CH2Attach
			+ (int)CH3Attach + (int)CH4Attach;
	return numberAttached >= 2;
}

unsigned int VacuumSensors::GetCH0() {
	printf("Send0 = %i",send[0]);
	send[0] = (uint8_t)0b00000001;
	printf("Send0 = %i",send[0]);
	send[1] = (uint8_t)0b10000000;
	send[2] = (uint8_t)0b00000000;
	printf("succeded in assigning send variable\n");
	unsigned int number = 0;
	vacuumSpi->Transaction(send, receive, 3);
	printf("Succeeded in the transaction\n");
	number = number + (((unsigned int) (receive[1] & 0b00000011)) << 8);
	number = number + ((unsigned int) receive[2]);
	printf("succeded in the parsing\n");

	return number;
}
unsigned int VacuumSensors::GetCH1() {
	printf("Getting CH1\n");

	send[0] = 0b00000001;
	send[1] = 0b10010000;
	send[2] = 0b00000000;

	unsigned int number = 0;

	vacuumSpi->Transaction(send, receive, 3);
	number = number + (((unsigned int) (receive[1] & 0b00000011)) << 8);
	number = number + ((unsigned int) receive[2]);

	return number;
}
unsigned int VacuumSensors::GetCH2() {
	printf("Getting CH2\n");

	send[0] = 0b00000001;
	send[1] = 0b10100000;
	send[2] = 0b00000000;

	unsigned int number = 0;

	vacuumSpi->Transaction(send, receive, 3);
	number = number + (((unsigned int) (receive[1] & 0b00000011)) << 8);
	number = number + ((unsigned int) receive[2]);

	return number;
}
unsigned int VacuumSensors::GetCH3() {
	printf("Getting CH3\n");

	send[0] = 0b00000001;
	send[1] = 0b10110000;
	send[2] = 0b00000000;

	unsigned int number = 0;

	vacuumSpi->Transaction(send, receive, 3);
	number = number + (((unsigned int) (receive[1] & 0b00000011)) << 8);
	number = number + ((unsigned int) receive[2]);

	return number;
}
unsigned int VacuumSensors::GetCH4() {
	printf("Getting CH4\n");

	send[0] = 0b00000001;
	send[1] = 0b11000000;
	send[2] = 0b00000000;

	unsigned int number = 0;

	vacuumSpi->Transaction(send, receive, 3);
	number = number + (((unsigned int) (receive[1] & 0b00000011)) << 8);
	number = number + ((unsigned int) receive[2]);

	return number;
}
unsigned int VacuumSensors::GetCH5() {
	printf("Getting CH5\n");

	send[0] = 0b00000001;
	send[1] = 0b11010000;
	send[2] = 0b00000000;

	unsigned int number = 0;

	vacuumSpi->Transaction(send, receive, 3);
	number = number + (((unsigned int) (receive[1] & 0b00000011)) << 8);
	number = number + ((unsigned int) receive[2]);

	return number;
}
unsigned int VacuumSensors::GetCH6() {
	send[0] = 0b00000001;
	send[1] = 0b11100000;
	send[2] = 0b00000000;

	unsigned int number = 0;

	vacuumSpi->Transaction(send, receive, 3);
	number = number + (((unsigned int) (receive[1] & 0b00000011)) << 8);
	number = number + ((unsigned int) receive[2]);

	return number;
}
unsigned int VacuumSensors::GetCH7() {
	send[0] = 0b00000001;
	send[1] = 0b11110000;
	send[2] = 0b00000000;

	unsigned int number = 0;

	vacuumSpi->Transaction(send, receive, 3);
	number = number + (((unsigned int) (receive[1] & 0b00000011)) << 8);
	number = number + ((unsigned int) receive[2]);

	return number;
}

bool VacuumSensors::CH0Attached() {
	bool CH0Attached = GetCH0()<threshold;
	printf("CH 0 Attached = %i", CH0Attached);
	return (CH0Attached);
}
bool VacuumSensors::CH1Attached() {
	bool CH1Attached = GetCH1()<threshold;
	printf("CH 1 Attached = %i", CH1Attached);
	return (CH1Attached);
}
bool VacuumSensors::CH2Attached() {
	bool CH2Attached = GetCH2()<threshold;
	printf("CH 2 Attached = %i", CH2Attached);
	return (CH2Attached);
}
bool VacuumSensors::CH3Attached() {
	bool CH3Attached = GetCH3()<threshold;
	printf("CH 3 Attached = %i", CH3Attached);
	return (CH3Attached);
}
bool VacuumSensors::CH4Attached() {
	bool CH4Attached = GetCH4()<threshold;
	printf("CH 4 Attached = %i", CH4Attached);
	return (CH4Attached);
}
bool VacuumSensors::CH5Attached() {
	return (GetCH5() < threshold);
}
bool VacuumSensors::CH6Attached() {
	return (GetCH6() < threshold);
}
bool VacuumSensors::CH7Attached() {
	return (GetCH7() < threshold);
}

double VacuumSensors::GetCH0Voltage() {
	return ((double) GetCH0()) * 5.0 / 1024.0;
}
double VacuumSensors::GetCH1Voltage() {
	return ((double) GetCH1()) * 5.0 / 1024.0;
}
double VacuumSensors::GetCH2Voltage() {
	return ((double) GetCH2()) * 5.0 / 1024.0;
}
double VacuumSensors::GetCH3Voltage() {
	return ((double) GetCH3()) * 5.0 / 1024.0;
}
double VacuumSensors::GetCH4Voltage() {
	return ((double) GetCH4()) * 5.0 / 1024.0;
}
double VacuumSensors::GetCH5Voltage() {
	return ((double) GetCH5()) * 5.0 / 1024.0;
}
double VacuumSensors::GetCH6Voltage() {
	return ((double) GetCH6()) * 5.0 / 1024.0;
}
double VacuumSensors::GetCH7Voltage() {
	return ((double) GetCH7()) * 5.0 / 1024.0;
}
VacuumSensors::~VacuumSensors() {
	// TODO Auto-generated destructor stub
}
