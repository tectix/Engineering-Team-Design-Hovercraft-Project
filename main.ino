#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Timers
unsigned long timer = 0;
unsigned long printTimer = 0;
float timeStep = 0.01; // Read data every 10 milliseconds
unsigned long printInterval = 1000; // Print data every 1000 milliseconds (1 second)

// Pitch, Roll, and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;


//Calibrating accels based on trials
float xAccCalib = -0.06;
float yAccCalib =0.0;
float zAccCalib = -0.06;

void setup() 
{
  Serial.begin(115200);

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  // Calibrate gyroscope. The calibration must be at rest.
  mpu.calibrateGyro();

  // Set threshold sensitivity. Default 3.
  mpu.setThreshold(3);
}

void loop()
{
  timer = millis();

  // Read normalized values from gyroscope
  Vector normGyro = mpu.readNormalizeGyro();

  // Read normalized values from accelerometer
  Vector normAccel = mpu.readRawAccel();

  // Calculate Pitch, Roll, and Yaw
  pitch = pitch + normGyro.YAxis * timeStep;
  roll = roll + normGyro.XAxis * timeStep;
  yaw = yaw + normGyro.ZAxis * timeStep;

  // Calculation of linear acceleration/forces 
  
  float linearAccelX = normAccel.XAxis /16384.0 +xAccCalib;
  float linearAccelY = normAccel.YAxis /16384.0 + yAccCalib;
  float linearAccelZ = normAccel.ZAxis /16384.0 +zAccCalib; 

  // Check if it's time to print the values (once every second)
  if (millis() - printTimer >= printInterval)
  {
    printTimer = millis(); // Reset the print timer

    // Output the calculated values
    Serial.print(" Pitch = ");
    Serial.print(pitch);
    Serial.print(" Roll = ");
    Serial.print(roll);  
    Serial.print(" Yaw = ");
    Serial.println(yaw);
    Serial.print(" Linear Accel X = ");
    Serial.print(linearAccelX);
    Serial.print(" Linear Accel Y = ");
    Serial.print(linearAccelY);
    Serial.print(" Linear Accel Z = ");
    Serial.println(linearAccelZ);
  }

  // Wait to complete the full timeStep period
  delay((timeStep*1000) - (millis() - timer));
}
