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
#define OBSTACLE_DISTANCE 35 // Distance in cm to detect an obstacle

float idealYaw =0.0;


float currentYaw  = 0;
float prevYaw  = 0;
float targetYaw = 0;
bool isTurning = false;
const float YAW_TOLERANCE = 5;
unsigned long timer = 0;
float timeStep = 0; // Read data every 10 milliseconds

float yawCalib = -0.06;

 NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
Servo serv;
float duration;
float distance =50 ;
 
int iterations =2;
 
unsigned long previousMillis = 0; 
const long servoInterval = 500; // Time for the servo to turn
float distanceLeft = 0;
float distanceRight = 0;

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}





enum State {
    STARTING,
    WAITING,
    TURNING_LEFT,
    MEASURING_LEFT,
    TURNING_RIGHT,
    MEASURING_RIGHT,
    DECIDING_DIRECTION,
    DONE_DECIDING,
    YAW_ADJUSTED,
    ADJUSTING_ANGLE,
    DONE_ADJUSTING
};
State currentState = STARTING;
int iteration = 0;
unsigned long lastLoopTime = 0;
unsigned long currentLoopTime = 0;


float prevDistance;
void setup() {
 initIMU();
  delay(150);
  initLiftFan();
  initThrustFan();
 //pinMode(TRIGGER_PIN, OUTPUT); // Sets the trigPin as an Output
// pinMode(ECHO_PIN, INPUT); // Sets the echoPin as an Input
  
   startLiftFan();
  Serial.begin (9600);
  serv.attach(SERVO_PIN);
 startThrustFan();
  servoMiddle();
  delay(150);
   
 
}
 
void loop() {
      if(iteration == 0){
        iteration++;
        currentState = WAITING;
        };
        

    unsigned long currentMillis = millis();
    unsigned long currentLoopTime = millis();
    if (lastLoopTime > 0) {
        timeStep = (currentLoopTime - lastLoopTime) / 1000.0;
    }

    Vector normGyro = mpu.readNormalizeGyro();
    currentYaw  = currentYaw  + normGyro.ZAxis * timeStep;

    Serial.print(" Yaw = "); Serial.println(currentYaw );

     distance = calculateDistance();
  //  delay(100);
    Serial.print("Distance = ");
    Serial.print(distance);
    Serial.println(" cm");
       
     if ( distance != 0.00 && distance < OBSTACLE_DISTANCE && currentState == WAITING  &&!isTurning) {

        // Stop hovercraft
      stopThrustFan();
      stopLiftFan();
     

        //   // Turn left
        currentState = TURNING_LEFT;
        
        servoLeft();
         previousMillis = currentMillis;
    }

      



    if ((currentYaw > idealYaw||currentYaw < idealYaw)   && !isTurning && currentState == WAITING) {
     // servoCustom( serv.read() -(currentYaw-idealYaw));
      float servoAngle=map(currentYaw,-90,90,0.0,180.0);
      serv.write(servoAngle);
    //  delay(150);
     // servoMiddle();

    }

  


 //   prevDistance = distance;

    switch (currentState) {

      //  case ADJUSTING_ANGLE:
      //       isTurning =true;
      //       float yawDifference = currentYaw - idealYaw ;
      //     adjustServoBasedOnYaw(yawDifference); 
      //     Serial.println("SENT NEW DATA -------------------------------");
      //     targetYaw = idealYaw;
      //     currentState = DONE_ADJUSTING;
          
      //       break;

      //   case DONE_ADJUSTING:
      //    if (abs(currentYaw- idealYaw) <= YAW_TOLERANCE) {


      //     Serial.println("HASS REACHED TARGET YAW -------------------------------");
      //             servoMiddle();
      //             delay(50);
      //              isTurning = false;
      //             currentState = WAITING;
                 
      //         }
      //         break;




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
              targetYaw = calculateTargetYaw(90);
                 servoCustom(45);
               // servoLeft();
            } else {
                targetYaw = calculateTargetYaw(-90);
                 servoCustom(135);
                 //servoRight();
            }
            startLiftFan();
            startThrustFan();
            
            // Restart hovercraft movement
            previousMillis = currentMillis;
            currentState = DONE_DECIDING;
            break;

          case DONE_DECIDING:
          if (currentMillis - previousMillis >= TURN_TIME+2000 ) {
         stopThrustFan();
          
          //delay(50);
                         //  recalibrateGyro();
                servoMiddle();
                currentState = YAW_ADJUSTED;
                previousMillis = currentMillis;
                isTurning = false;
            }
            break;
          // if (hasReachedTargetYaw()) {
          //   Serial.println("Reached correct yaw");
          //   idealYaw = currentYaw;
          //   stopThrustFan();
          //   stopLiftFan();
          //   delay(50);
          //                 //  recalibrateGyro();
          //         servoMiddle();
          //         currentState = YAW_ADJUSTED;
          //         previousMillis = currentMillis;
          //         isTurning = false;
          //     }
          //     break;
        
          case YAW_ADJUSTED:
            startLiftFan();
            startThrustFan();
            currentState= WAITING;
            break;



             case ADJUSTING_ANGLE:
            isTurning =true;
            float yawDifference = currentYaw - idealYaw ;
          adjustServoBasedOnYaw(yawDifference); 
          Serial.println("SENT NEW DATA -------------------------------");
          targetYaw = idealYaw;
          currentState = DONE_ADJUSTING;
          //currentState = WAITING;

            break;


              case DONE_ADJUSTING:
               Serial.println("CURRENT STATE IS DONE ADJUSTING ANGLE");
            //  delay(150);
               Serial.println("HASS REACHED TARGET YAW -------------------------------");
                  servoMiddle();
               //   delay(50);
                   isTurning = false;
                  currentState = WAITING;
                 
              
              break;

       
        default:
            break;
    }

  lastLoopTime = currentLoopTime;
  prevYaw = currentYaw;
}


 float calculateDistance(){
   duration = sonar.ping_median(iterations);
   return ( (duration / 2) * 0.0343);
 }



  float calculateDistance2(){

     // Clears the trigPin
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(ECHO_PIN, HIGH);
  // Calculating the distance
  return duration * 0.034 / 2;
 

 }

bool weirdAngle()  {
  if(abs(currentYaw-prevYaw) >= YAW_TOLERANCE ){
          return true; 
        }
   return false;

    }

  
void adjustAngle(int angle)  {
  servoCustom(angle) ;
  delay(150);
  servoMiddle();
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
   // currentYaw = normalizeYaw(currentYaw); // Correctly normalize the yaw value
}

float calculateTargetYaw(float turnAngle) {
    float newYaw = currentYaw + turnAngle;
    return newYaw; // Adjust for wrapping and return
}

bool hasReachedTargetYaw() {
    return abs(currentYaw - targetYaw) <= YAW_TOLERANCE;
}

void stopMovement() {
    stopLiftFan();
    stopThrustFan();
    //servoMiddle();
}
 
void calibrateYaw() {
if (currentYaw >= 5 || currentYaw <= -5) {
  Serial.print("Calibrated :"); Serial.print(-currentYaw);
    servoCustom(-currentYaw);
};
}

void recalibrateGyro() {
  currentYaw = 0; // Reset yaw after calibration
    mpu.calibrateGyro();
   
}

void adjustServoBasedOnYaw(float yawDiff) {
    int adjustment = (yawDiff > 0) ? -1 : 1;
    //servoLeft();
    servoCustom(serv.read() + adjustment*yawDiff);
}

float normalizeYaw(float yaw) {
    // Normalize yaw to be within 0 to 360 degrees
    yaw = fmod(yaw, 360);
    if (yaw < 0) {
        yaw += 360;
    }

    // Convert to 0 to 180 degrees
    if (yaw > 180) {
        yaw = 360 - yaw;
    }

    return yaw;
}
