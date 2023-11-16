#pragma once
#ifndef SENSORS_H
#define SENSORS_H
#include <Arduino.h>
#include "TWI.h"

/*Connections:
* P2 servo motor
* P13 front US Sensor
* P11 lift fan
* P3 thrust fan
* P17 IMU
*/


#define IN_PULSE (1 << 7)
#define DATA_READY (1 << 6)


typedef struct {
	uint16_t pulseStart;
	uint16_t pulseEnd;
	uint32_t pulseLength;
	uint8_t semaphore;
} USSensorData;


void initIMU();
void initUSS(USSensorData* frontSensorData);
void calibrateIMU();

#endif