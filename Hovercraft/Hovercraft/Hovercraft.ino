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

float pitch = 0;
float roll = 0;
float yaw = 0;
unsigned long timer = 0;
unsigned long printTimer = 0;
float timeStep = 0.1; // Read data every 10 milliseconds

float yawCalib = -0.06;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
Servo serv;
float duration, distance;
 
int iterations = 3;
 
void setup() {
  Serial.begin (9600);
  serv.attach(SERVO_PIN);
  initIMU();
  //initThrustFan();
  //startThrustFan();
  servoMiddle();
}
 
void loop() {

    Vector normGyro = mpu.readNormalizeGyro();

    // Read normalized values from accelerometer

    yaw = yaw + normGyro.ZAxis * timeStep;


    Serial.print(" Yaw = ");
    Serial.println(yaw);
  // duration = sonar.ping_median(iterations);
  
  // // Determine distance from duration
  // // Use 343 metres per second as speed of sound
  
  distance = calculateDistance();
  
  // Send results to Serial Monitor
  Serial.print("Distance = ");
  if (distance >= 400 || distance <= 2) {
    Serial.println("Out of range");
  }
  else {
    Serial.print(distance);
    Serial.println(" cm");

    if(distance < 12){
    Serial.print("WARNING COLLISION");
      
      delay(5000);
    }
    delay(10);
  }
  startThrustFan();
  delay(10);
}


 float calculateDistance(){
   duration = sonar.ping_median(iterations);
   return ( (duration / 2) * 0.0343);
 }



void servoLeft(){
serv.write(0);
}

void servoRight(){
serv.write(185);
}

void servoMiddle(){
  serv.write(90);
}

