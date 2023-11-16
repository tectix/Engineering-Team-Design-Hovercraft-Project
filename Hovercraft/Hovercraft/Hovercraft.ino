#include <Arduino.h>
#include "lift.h"
#include "sensors.h"



void setup() {
  initLift();
  delay(500);

}

void loop() {

    setLift(10);
    delay(3000);
    setLift(50);
    delay(10000);
    setLift(100);



}


void setLift(uint8_t lift) {
  OCR1A = (lift) * 65535 / 100;
}