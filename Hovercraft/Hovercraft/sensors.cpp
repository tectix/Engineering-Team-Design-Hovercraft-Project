#include "sensors.h"



MPU6050 mpu;


void initIMU() {
    while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_4G))
    {
        Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
        delay(500);
    }

    // Calibrate gyroscope. The calibration must be at rest.
    mpu.calibrateGyro();

    // Set threshold sensitivity. Default 3.
    mpu.setThreshold(3);
}

