#pragma once
#include <Servo.h>
#include "pins.h"



extern Servo servo;

void moveStepper(int steps);
void dropVictims(char side, char vic);
void drop(bool onLeft);