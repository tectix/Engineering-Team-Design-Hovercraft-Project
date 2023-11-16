#include <Arduino.h>
#include "TWI.h"
#include "lift.h"
#include "sensors.h"





void moveForwardTest();
void turnLeft();
void turnRight();
void setThrust(uint8_t strength);
void setLift(uint8_t lift);

typedef struct {
  uint32_t ussData = 100;
  int16_t yawRate = 0;
} SensorData;

volatile uint8_t semaphore = 0;
volatile uint8_t timer2cnt = 0;



void setup() {
  Serial.begin(9600); // comment out for actual run
  delay(50);
  Serial.println("start");
  initTiming();
  delay(75);
  initLift();
  delay(500);
  Serial.println("starting lift");
  setLift(10);
  delay(1000);
  Serial.println("Half lift");
  setLift(30);
  delay(1000);
  Serial.println("Fulll lift");
  setLift(75);
  delay(1000);
}

void loop() {

}


void setLift(uint8_t lift) {
  OCR1A = (lift) * 65535 / 100;
}