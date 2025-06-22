#pragma once
#include <utility>
#include "Wire.h"
#include "dropper.h"
#include "motor.h"
#include "led.h"
#include "pins.h"
#include "tof.h"

extern volatile bool interrupted;
extern int restartPi;
extern bool doVictims;
extern bool goingForward;
extern bool turning;
void interruptFunc();
void commBegin();