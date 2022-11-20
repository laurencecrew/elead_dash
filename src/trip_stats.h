#ifndef TRIP_STATS_H
#define TRIP_STATS_H

#include <Arduino.h>
#include <STM32RTC.h>

#define WHEEL_CIRC    1360    // Wheel circumference; mm

typedef struct 
{
  uint32_t distance_mm = 0;     // cumulative trip distance in mm
  int32_t watt_s_x100 = 0;      // cumulative Watt/seconds * 100
  uint16_t avg_speed_x10 = 0;   // average speed in km/h * 10
  STM32RTC& trip_time = STM32RTC::getInstance();
} Trip_stats_t;

extern Trip_stats_t trip_stats;

void TRIP_STATS_init ();
void TRIP_STATS_reset ();

#endif

