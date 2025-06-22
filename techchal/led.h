#pragma once
#include <Arduino.h>

void blink(int amt = 5, int length = 500);
void ledInit();

// #define NUM_LEDS 2
// #define COLOR_ORDER NEO_RGB
// int8_t pinss[8] = {23, -1, -1, -1, -1, -1, -1, -1};


// Adafruit_NeoPXL8 ledss(NUM_LEDS, pinss, COLOR_ORDER);