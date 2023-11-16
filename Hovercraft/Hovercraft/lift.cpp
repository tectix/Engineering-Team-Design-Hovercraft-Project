#include "lift.h"
#include <Arduino.h>


void initLift() {
	// Set up Timer 1 for Fast PWM
	TCCR1A = 0x82; // Fast PWM Mode, non-inverting
	TCCR1B = 0x03; // Prescaler = 64
	OCR1A = 0; // Initial duty cycle = 0%
	DDRB |= (1 << 1);
}