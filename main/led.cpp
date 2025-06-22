#include "led.h"

//   int NUM_LEDS = 2;

// int8_t pinss[8] = {23, -1, -1, -1, -1, -1, -1, -1};


//  Adafruit_NeoPXL8 ledss(NUM_LEDS, pinss, NEO_RGB);

void ledInit() {

  pinMode(6, OUTPUT);
}


void blink(int amt, int length) {

  for(int i = 0; i < amt; i++) {
  digitalWrite(6, HIGH);
  delay(length);

  digitalWrite(6, LOW);
  delay(length);
  }

}