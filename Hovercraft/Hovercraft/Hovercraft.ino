#include <Arduino.h>
#include <MPU6050.h>
#include <Servo.h>
#include "NewPing.h"
#include "sensors.h"
#include "fans.h"
 

#define TRIGGER_PIN  13
#define ECHO_PIN     3
#define MAX_DISTANCE 400
#define SERVO_PIN 6
#define FORWARD_TIME 4000 // Time in milliseconds to move forward 
#define TURN_TIME 500 // Estimated time to complete a U-turn 
#define OBSTACLE_DISTANCE 30 // Distance in cm to detect an obstacle


float currentYaw  = 0;
float targetYaw = 0;
bool isTurning = false;
const float YAW_TOLERANCE = 5;
unsigned long timer = 0;
float timeStep = 0.1; // Read data every 10 milliseconds

float yawCalib = -0.06;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
Servo serv;
float duration, distance;
 
int iterations = 3;
 
unsigned long previousMillis = 0; 
const long servoInterval = 500; // Time for the servo to turn
float distanceLeft = 0;
float distanceRight = 0;




enum State {
    WAITING,
    TURNING_LEFT,
    MEASURING_LEFT,
    TURNING_RIGHT,
    MEASURING_RIGHT,
    DECIDING_DIRECTION,
    DONE_DECIDING,
    ADJUST_IMU
};
State currentState = WAITING;


void setup() {
  Serial.begin (9600);
  serv.attach(SERVO_PIN);
  initIMU();
  initLiftFan();
  
  initThrustFan();
  startThrustFan();
  startLiftFan();
  servoMiddle();
}
 
void loop() {

    unsigned long currentMillis = millis();
   // Vector normGyro = mpu.readNormalizeGyro();
  //  currentYaw  = currentYaw  + normGyro.ZAxis * timeStep;
   // Serial.print(" Yaw = "); Serial.println(currentYaw );
    updateYaw();
    distance = calculateDistance();
    Serial.print("Distance = ");
    Serial.print(distance);
    Serial.println(" cm");


     if (distance < OBSTACLE_DISTANCE && currentState == WAITING  &&!isTurning) {
        // Stop hovercraft
        stopThrustFan();
        stopLiftFan();

        //   // Turn left
         currentState = TURNING_LEFT;
        
        servoLeft();
        previousMillis = currentMillis;
    }

    switch (currentState) {
        case TURNING_LEFT:
            if (currentMillis - previousMillis >= servoInterval) {
                currentState = MEASURING_LEFT;
                previousMillis = currentMillis;
            }
            break;

        case MEASURING_LEFT:
            if (currentMillis - previousMillis >= servoInterval) {
                distanceLeft = calculateDistance();
                currentState = TURNING_RIGHT;
                servoRight();
                previousMillis = currentMillis;
            }
            break;

        case TURNING_RIGHT:
            if (currentMillis - previousMillis >= servoInterval) {
                currentState = MEASURING_RIGHT;
                previousMillis = currentMillis;
            }
            break;

        case MEASURING_RIGHT:
            if (currentMillis - previousMillis >= servoInterval) {
                distanceRight = calculateDistance();
                currentState = DECIDING_DIRECTION;
            }
            break;

        case DECIDING_DIRECTION:
             
            if (distanceLeft > distanceRight) {
                servoCustom(45);
            } else {
                servoCustom(135);
            }
            startLiftFan();
            startThrustFan();
            
            // Restart hovercraft movement
            previousMillis = currentMillis;
            currentState = DONE_DECIDING;
            break;

          case DONE_DECIDING:
          if (currentMillis - previousMillis >= servoInterval+ 1500) {
                servoMiddle();
                currentState = WAITING;
                previousMillis = currentMillis;
                isTurning = false;
            }
            break;
        

        case ADJUST_IMU:
         break;

        default:
            break;
    }







}


 float calculateDistance(){
   duration = sonar.ping_median(iterations);
   return ( (duration / 2) * 0.0343);
 }



void servoLeft(){
serv.write(0);
}

void servoRight(){
serv.write(183);
}

void servoCustom(int angle){
serv.write(angle);
}

void servoMiddle(){
  serv.write(89);
}



void updateYaw() {
    Vector normGyro = mpu.readNormalizeGyro();
    currentYaw += normGyro.ZAxis * timeStep;
    currentYaw = normalizeYaw(currentYaw); // Correctly normalize the yaw value
}

float calculateTargetYaw(float turnAngle) {
    float newYaw = currentYaw + turnAngle;
    return normalizeYaw(newYaw); // Adjust for wrapping and return
}

bool hasReachedTargetYaw() {
    return abs(normalizeYaw(currentYaw) - targetYaw) < YAW_TOLERANCE;
}

float normalizeYaw(float yaw) {
    while (yaw < 0) yaw += 360;
    while (yaw >= 360) yaw -= 360;
    return yaw;
}

void stopMovement() {
    stopThrustFan();
    stopLiftFan();
    servoMiddle();
}

void recalibrateGyro() {
    mpu.calibrateGyro();
    currentYaw = 0; // Reset yaw after calibration
}


//  case TURNING_LEFT:
//             if (!isTurning) {
//                 targetYaw = calculateTargetYaw(90); // 90 degrees left turn
//                 isTurning = true;
//                 servoLeft(); // Start turning left
//             } else if (hasReachedTargetYaw()) {
//                 currentState = MEASURING_LEFT;
//                 isTurning = false;
//                 previousMillis = currentMillis;
//                 servoMiddle(); // Stop turning
//             }
//             break;

//         case TURNING_RIGHT:
//             if (!isTurning) {
//                 targetYaw = calculateTargetYaw(-90); // 90 degrees right turn
//                 isTurning = true;
//                 servoRight(); // Start turning right
//             } else if (hasReachedTargetYaw()) {
//                 currentState = MEASURING_RIGHT;
//                 isTurning = false;
//                 previousMillis = currentMillis;
//                 servoMiddle(); // Stop turning
//             }
//             break;



//         case DECIDING_DIRECTION:
//             // ... [Decision logic]
//             startLiftFan();
//             startThrustFan();
//             currentState = DONE_DECIDING;
//             break;
