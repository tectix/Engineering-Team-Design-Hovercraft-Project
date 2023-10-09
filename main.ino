#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
MPU6050 mpu;


Servo sg90;
int servo_pin = 6;
// Timers
unsigned long timer = 0;
unsigned long printTimer = 0;
float timeStep = 0.01; // Read data every 10 milliseconds
unsigned long printInterval = 500; // Print data every 1000 milliseconds (1 second)

// Pitch, Roll, and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;


//Calibrating accelerations based on trials
float xAccCalib = -0.01;
float yAccCalib =0.0;
float zAccCalib = -0.06;
float pitchCalib = -0.06;
float rollCalib = -0.06;
float yawCalib = -0.06;

//LSB sensitivity constants for full scale range
const float MPU6050_SENS_FOR_RANGE_2 = 16384.0;
const float MPU6050_SENS_FOR_RANGE_4 = 8192.0;
const float MPU6050_SENS_FOR_RANGE_8 = 4096.0;
const float MPU6050_SENS_FOR_RANGE_16 = 2048.0;

float lsbSensitivity = MPU6050_SENS_FOR_RANGE_2;



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


void setup() 
{
  sg90.attach(servo_pin);
  Serial.begin(115200);

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensitivity. Default 3.
  // If you don't want to use threshold, comment this line or set 0.
  mpu.setThreshold(3);
}

void loop()
{
  timer = millis();

  // Read normalized values from gyroscope
  Vector normGyro = mpu.readNormalizeGyro();

  // Read normalized values from accelerometer
  Vector normAccel = mpu.readRawAccel();

  // Calculate Pitch, Roll, and Yaw deg/s * s = deg
  pitch = pitch + normGyro.YAxis * timeStep;
  roll = roll + normGyro.XAxis * timeStep;
  yaw = yaw + normGyro.ZAxis * timeStep;
  
  int servoMotion = constrain(yaw + 90, 0, 180);
  sg90.write(servoMotion);
  // Calculate linear accelerations
  float linearAccelX = normAccel.XAxis /lsbSensitivity +xAccCalib;
  float linearAccelY = normAccel.YAxis /lsbSensitivity + yAccCalib;
  float linearAccelZ = normAccel.ZAxis /lsbSensitivity +zAccCalib; 

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
