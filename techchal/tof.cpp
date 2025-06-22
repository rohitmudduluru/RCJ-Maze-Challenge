#include "tof.h"


Adafruit_VL53L0X tof = Adafruit_VL53L0X();


void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void tofInit() {
  //tof = Adafruit_VL53L0X();
  Serial.println("Starting tof init");
  for (int i : tofs) {
    tcaselect(i);
    Serial.println(i);
    tof.begin();
  }
}


double readTOF(int num) {
  tcaselect(num);
  VL53L0X_RangingMeasurementData_t measure;
  tof.rangingTest(&measure, false);  // pass in 'true' to get debug data printout!
  if (measure.RangeStatus == 4) {
    return 8196;
  }
  return measure.RangeMilliMeter * 6.25 / 80;
}

void printTOFs() {
  Serial.print("Front: ");
  Serial.print(readTOF(FRONT_TOF));
  Serial.print("; Back: ");
  Serial.print(readTOF(BACK_TOF));
  Serial.print("; Left: ");
  Serial.print(readTOF(LEFT_TOF));
  Serial.print("; Right: ");
  Serial.println(readTOF(RIGHT_TOF));
}