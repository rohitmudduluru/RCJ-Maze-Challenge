#include "lidar.h"


#define RES8 VL53L7CX_RESOLUTION_8X8
VL53L7CX sensor_vl53l7cx_top(&Wire1, 0);


void lidarInit() {

  sensor_vl53l7cx_top.begin();
  sensor_vl53l7cx_top.init_sensor((uint8_t)0x52);
  sensor_vl53l7cx_top.vl53l7cx_set_resolution(RES8);
  sensor_vl53l7cx_top.vl53l7cx_start_ranging();
}

double getOffsetDueToObject() {
  VL53L7CX_ResultsData Results;
  uint8_t NewDataReady = 0;
  uint8_t status;

  do {
    status = sensor_vl53l7cx_top.vl53l7cx_check_data_ready(&NewDataReady);
  } while (!NewDataReady);

  if (status != 0) {
    Serial.println("ERROR status get_ranging_data");
    while (1)
      ;
  }

  if ((!status) && (NewDataReady != 0)) {
    status = sensor_vl53l7cx_top.vl53l7cx_get_ranging_data(&Results);
    //print_result(&Results);
  }

  if (status != 0) {
    Serial.println("ERROR status get_ranging_data");
    while (1)
      ;
  }

  //Serial.println("Printing Data: ");

  uint8_t res = VL53L7CX_RESOLUTION_8X8;  //(uint8_t)64U

  uint8_t number_of_zones = res;

  double addition = 0;

  uint8_t zones_per_line = (number_of_zones == 16) ? 4 : 8;
  int i, j, k, l;
  char buff[255];


  Serial.println("\n");
  for (j = 0; j < number_of_zones; j += zones_per_line) {

    //-> NB_Targets thing is just 1, and doesnt seem to have other values other than 1...
    for (l = 0; l < VL53L7CX_NB_TARGET_PER_ZONE; l++) {
      for (k = (zones_per_line - 1); k >= 0; k--) {

        // this will indicate if ANYTHING was detected in that zone
        if (Results.nb_target_detected[j + k] > 0) {
          //sprintf(buff, "%5d", (long)Results.distance_mm[(VL53L7CX_NB_TARGET_PER_ZONE * (j+k)) + l]);
          //Serial.print(buff);

          // check distance of area that was detected and display the zone
          if ((long)Results.distance_mm[(VL53L7CX_NB_TARGET_PER_ZONE * (j + k)) + l] < 60) {
            Serial.print("Close detect: ");
            Serial.print((VL53L7CX_NB_TARGET_PER_ZONE * (j + k)) + l);
            switch ((VL53L7CX_NB_TARGET_PER_ZONE * (j + k)) + l) {
              case 2:
                addition += 8;
              case 3:
                addition += 12;
              case 4:
                addition += 15;
              case 5:
                addition -= 15;
              case 6:
                addition -= 12;
              case 7:
                addition -= 8;
            }
          }
        } else {
          //sprintf(buff, "%10s", "   x   ");
          //Serial.print(buff);
        }
      }

    }  // end L loop
    // just to look nice
    /*
    for (i = 0; i < zones_per_line; i++)
      SerialPort.print(" -----------------");
    SerialPort.print("\n");
    */
  }
  return addition;
}