#pragma once
#include "robot.h"
#include "pins.h"
#include "comm.h"

#define kP 5
#define kD 1

extern volatile int encR; //front right
void enc_updateR();
extern volatile int encL; //front left
void enc_updateL();

void lmotors(int speed);
void rmotors(int speed);
void forward(int speed);
// void forward(int lspeed, int rspeed, bool x);
void backward(int speed);
void stop_motors();

void forwardCm(int speed, int cm);
void backwardCm(int speed, int cm);
void testForward();
// void backtrack(int speed);

int cmToEnc(double cm);
double encToCm(int enc);
void printEncs();

class Motor {
public:
  uint8_t fpin;
  uint8_t rpin;
  double trim; // so motors move at same speed
  void speed(int percent);
  Motor(int f, int r, double t = 1);
};

// extern Motor m;

extern Motor frontLeft;
extern Motor frontRight;
extern Motor backLeft;
extern Motor backRight;