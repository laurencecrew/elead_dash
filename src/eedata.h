#ifndef EEDATA_H
#define EEDATA_H

#include "FlashStorage_STM32F1.hpp"
#include "trip_stats.h"
#include "debug.h"

#define BUFFER_SIZE          127 // for 7-byte data object, 127 + (127*7) is required i.e. 1016 bytes
#define ADDR_BUFFER_START    0 // start index of address buffer NOTE only tested starting at 0
#define DATA_BUFFER_START    ADDR_BUFFER_START + BUFFER_SIZE

// save trip stats and gauge value in dash data
typedef struct
{
  uint8_t soc;            // state of charge for battery gauge
  int16_t distance_100m;  // cumulative trip distance, 100m resolution
  int16_t watt_hrs;       // cumulative Watt/hours
  uint8_t hours;          // trip time to nearest minute
  uint8_t minutes;
} dashData_t;

void EEDATA_load (dashData_t *data);
void EEDATA_save (dashData_t *data);

#endif
