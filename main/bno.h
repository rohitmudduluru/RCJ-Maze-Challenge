#pragma once
#include <Adafruit_BNO055.h>

extern Adafruit_BNO055 bno;

int getBNO();
int roundAngle(int angle);
float getTilt();