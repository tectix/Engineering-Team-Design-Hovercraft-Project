#include "fans.h"
#include <Arduino.h>




void startThrustFan() {
    PORTD |= 1 << PD7;
}

void stopThrustFan() {
    PORTD &= ~(1 << PD7);
}

void startLiftFan() {
    PORTD |= 1 << PD4;
}

void stopLiftFan() {
    PORTD &= ~(1 << PD4);
}


void initThrustFan() {
    DDRD |= 1 << PD7;
}
void initLiftFan() {
    DDRD |= 1 << PD4;
}