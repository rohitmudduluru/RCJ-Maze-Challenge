#include "bno.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Get BNO x rotation
int getBNO() {
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;
}

// Rounds angle to the nearest 90 [0 360]
int roundAngle(int angle) {
  angle %= 360;
  if (angle < 0) angle += 360;
  return angle % 90 > 45 ? (angle + 90) - (angle % 90) : angle - (angle % 90);
}

// Get BNO z rotation
float getTilt() {
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.z;
}