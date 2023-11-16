#pragma once
#ifndef TIMING_H
#define TIMING_H
#include <Arduino.h>

#define IMU_SEMAPHORE (1 << 7) //period of 125us, I2C clock rate of 400KHz. 
#define TWI_DATA_READY_SEMAPHORE  (1 << 5)
#define TWI_BUSY_SEMAPHORE (1 << 4)
#define CONTROL_SEMAPHORE (1 << 3)

#define TIMER2_CNTIN (255 - 125) //125 counts with a prescalar of 1 gives 125us with a 1MHz io clock

#define SERVO_MIDDLE (88)
#define SERVO_LEFT (32)
#define SERVO_RIGHT (150)
#define SERVO_START (80)

void initTiming();
#endif