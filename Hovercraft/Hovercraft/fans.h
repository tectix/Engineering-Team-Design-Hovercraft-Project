#pragma once
#ifndef FANS_H
#define FANS_H
#include <Arduino.h>


void startThrustFan();
void stopThrustFan();
void startLiftFan();
void stopLiftFan();
void initThrustFan();
void initLiftFan();
#endif