#ifndef SENSORS_H
#define SENSORS_H
#include <MPU6050.h>
#include <Arduino.h>






//LSB sensitivity constants for full scale range
const float MPU6050_SENS_FOR_RANGE_2 = 16384.0;
const float MPU6050_SENS_FOR_RANGE_4 = 8192.0;
const float MPU6050_SENS_FOR_RANGE_8 = 4096.0;
const float MPU6050_SENS_FOR_RANGE_16 = 2048.0;

extern MPU6050 mpu;
// Initializing the mpu function, choose the appropriate scale/ range. the options are
    // MPU6050_SCALE_2000DPS       - 250 deg/sec => sensitivity 16.4.0
    // MPU6050_SCALE_1000DPS       - 1000 deg/sec => sensitivity 328.0
    // MPU6050_SCALE_500DPS        - 500 deg/sec => sensitivity 655.0
    // MPU6050_SCALE_250DPS        - 250 deg/sec => sensitivity 131.0

// Change the LSB sensitivity constants when choosing a range
    // MPU6050_RANGE_16G             = 16g = MPU6050_SENS_FOR_RANGE_16
    // MPU6050_RANGE_8G              = 8g  = MPU6050_SENS_FOR_RANGE_8
    // MPU6050_RANGE_4G              = 4g  = MPU6050_SENS_FOR_RANGE_4
    // MPU6050_RANGE_2G              = 2g  = MPU6050_SENS_FOR_RANGE_2



/*all connections
* P14 left IR sensor
* P16 right IR sensor
* P2 servo motor
* P13 front US Sensor
* P11 lift fan
* P3 thrust fan
* P17 IMU
*/




//init function for IMU
void initIMU();

void calibrateIMU();

#endif
