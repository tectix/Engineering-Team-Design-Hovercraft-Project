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
#define FORWARD_TIME 6000 // Time in milliseconds to move forward 
#define TURN_TIME 3000 // Estimated time to complete a U-turn 
#define OBSTACLE_DISTANCE 15 // Distance in cm to detect an obstacle


float currentYaw  = 0;
float targetYaw = 0;
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




//Competition states
// enum State {
//     WAITING,
//     TURNING_LEFT,
//     MEASURING_LEFT,
//     TURNING_RIGHT,
//     MEASURING_RIGHT,
//     DECIDING_DIRECTION,
//     DONE_DECIDING
// };
// State currentState = WAITING;

//Demo states
enum State {
  WAITING,
    MOVING_FORWARD,
    
    TURNING,
    TURNING2,
    COMPLETED,
    STOPPING,
    MOVING_BACK
};
State currentState = MOVING_FORWARD;





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
    Vector normGyro = mpu.readNormalizeGyro();
    currentYaw  = currentYaw  + normGyro.ZAxis * timeStep;
    Serial.print(" Yaw = "); Serial.println(currentYaw );
    distance = calculateDistance();
    Serial.print("Distance = ");
    Serial.print(distance);
    Serial.println(" cm");


    //  if (distance < OBSTACLE_DISTANCE && currentState == WAITING) {
    //     // Stop hovercraft
    //     stopThrustFan();
    //     stopLiftFan();

    //       // Turn left
    //      currentState = TURNING_LEFT;
        
    //     servoLeft();
    //     previousMillis = currentMillis;
    // }

    // switch (currentState) {
    //     case TURNING_LEFT:
    //         if (currentMillis - previousMillis >= servoInterval) {
    //             currentState = MEASURING_LEFT;
    //             previousMillis = currentMillis;
    //         }
    //         break;

    //     case MEASURING_LEFT:
    //         if (currentMillis - previousMillis >= servoInterval) {
    //             distanceLeft = calculateDistance();
    //             currentState = TURNING_RIGHT;
    //             servoRight();
    //             previousMillis = currentMillis;
    //         }
    //         break;

    //     case TURNING_RIGHT:
    //         if (currentMillis - previousMillis >= servoInterval) {
    //             currentState = MEASURING_RIGHT;
    //             previousMillis = currentMillis;
    //         }
    //         break;

    //     case MEASURING_RIGHT:
    //         if (currentMillis - previousMillis >= servoInterval) {
    //             distanceRight = calculateDistance();
    //             currentState = DECIDING_DIRECTION;
    //         }
    //         break;

    //     case DECIDING_DIRECTION:
    //         if (distanceLeft > distanceRight) {
    //             servoLeft();
    //         } else {
    //             servoRight();
    //         }
    //          startLiftFan();
    //         startThrustFan(); // Restart hovercraft movement
    //         previousMillis = currentMillis;
    //         currentState = DONE_DECIDING;
    //         break;

    //       case DONE_DECIDING:
    //       if (currentMillis - previousMillis >= servoInterval + 500) {
    //             servoMiddle();
    //             currentState = WAITING;
    //             previousMillis = currentMillis;
    //         }
    //         break;

    //     default:
    //         break;
    // }






    //SWitch logic for the demo
     switch (currentState) {
        case MOVING_FORWARD:
            if (currentMillis - previousMillis >= FORWARD_TIME) {
                //stopLiftFan();
               // stopThrustFan();
                currentState = TURNING;
                servoLeft(); // Initiate U-turn to the right
        
                previousMillis = currentMillis;
            }
            break;

        case TURNING:
            if (currentMillis - previousMillis >= TURN_TIME -1500) {
               // startLiftFan();
               // startThrustFan();
                currentState = TURNING2;
                servoMiddle(); // Reset servo to move straight back
               // startThrustFan();
                previousMillis = currentMillis;
            }
            break;

            case TURNING2:
            if (currentMillis - previousMillis >= TURN_TIME -2500) {
                currentState = MOVING_BACK;
                servoMiddle(); // Reset servo to move straight back
               // startThrustFan();
                previousMillis = currentMillis;
            }
            break;

        case MOVING_BACK:
            if (distance < OBSTACLE_DISTANCE) {
                stopThrustFan();
                stopLiftFan();
                currentState = STOPPING;
            }
            break;

        case STOPPING:
            // Hovercraft has stopped due to an obstacle
            currentState = COMPLETED;
            break;

        case COMPLETED:
    
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


void resetGyro() {
    currentYaw  = 0;
    mpu.calibrateGyro();
}



//IMU verification inside the decision tree
  // switch (currentState) {
  //       case TURNING_LEFT:
  //           if (!isTurning) {
  //               targetYaw = currentYaw - 90; // Adjust for 90 degrees turn
  //               isTurning = true;
  //           }
  //           if (currentYaw <= targetYaw) {
  //               currentState = MEASURING_LEFT;
  //               isTurning = false;
  //               previousMillis = currentMillis;
  //           }
  //           break;

  //       case TURNING_RIGHT:
  //           if (!isTurning) {
  //               targetYaw = currentYaw + 90; // Adjust for 90 degrees turn
  //               isTurning = true;
  //           }
  //           if (currentYaw >= targetYaw) {
  //               currentState = MEASURING_RIGHT;
  //               isTurning = false;
  //               previousMillis = currentMillis;
  //           }
  //           break;

