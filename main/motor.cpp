#include "motor.h"

//Motor m(11,10);

Motor frontRight(8, 9, .935);
Motor frontLeft(11, 10, 339.1/351.1 * .935);
Motor backLeft(12, 13, 339.1/351.1);
Motor backRight(15, 14, .935);

volatile int encR = 0;
volatile int encL = 0;
void enc_updateR() {
  if (digitalRead(ENC_PIN_R) == HIGH) {
    encR--;
  } else {
    encR++;
    if (restartPi > 0 && goingForward) restartPi--;
  }
}
void enc_updateL() {
  if (digitalRead(ENC_PIN_L) == HIGH) {
    encL++;
  } else {
    encL--;
  }
}

void printEncs() {
  Serial.print("Left Encoder: ");
  Serial.print(encL);
  Serial.print("; Right Encoder: ");
  Serial.println(encR);
}

void lmotors(int speed) {
  if (speed > 100) speed = 100;
  if (speed < -100) speed = -100;
  frontLeft.speed(speed);
  backLeft.speed(speed);
}
void rmotors(int speed) {
  if (speed > 100) speed = 100;
  if (speed < -100) speed = -100;
  frontRight.speed(speed);
  backRight.speed(speed);
}

void forward(int speed) {
  lmotors(speed);
  rmotors(speed);
}


void backward(int speed) {
  lmotors(-speed);
  rmotors(-speed);
}
void stop_motors() {
  lmotors(0);
  rmotors(0);
}

void forwardCm(int speed, int cm) {
  encR = 0;
  while (encToCm(encR) < cm) {
    forward(speed);
  }
}
void backwardCm(int speed, int cm) {
  encR = 0;
  while (encToCm(encR) > -cm) {
    backward(speed);
  }
}


// void backtrack(speed) {
//   while(enc > 0) {
//     backward(speed);
//   }
// }

int cmToEnc(double cm) {
  return cm * ENC_PER_ROT / (PI * WHEELDIA);
}
double encToCm(int enc) {
  return enc * PI * WHEELDIA / ENC_PER_ROT;
}


void testForward() {
  encR = 0;
  while (encR < ENC_PER_ROT) {
    forward(40);
  }
  stop_motors();
  delay(1000);
}

void Motor::speed(int percent) {
  int speed = trim * abs(percent) * 255 / 100;
  if (percent > 0) {
    analogWrite(fpin, speed);
    analogWrite(rpin, 0);
  } else {
    analogWrite(fpin, 0);
    analogWrite(rpin, speed);
  }
}
Motor::Motor(int f, int r, double t) {
  fpin = f;
  rpin = r;
  trim = t;
}