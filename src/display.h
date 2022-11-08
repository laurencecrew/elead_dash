#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include "bitmaps.h"
#include "votol_em.h"
#include "dalybms.h"
#include "trip_stats.h"

/*
    Note this version uses 0.66" 64x48 SPI SSD1306 OLED displays x 2
*/

#define SCREEN_WIDTH    64   // OLED display width, in pixels
#define SCREEN_HEIGHT   48   // OLED display height, in pixels
#define COLUMN_OFFSET   32   // Offset of first column from 0, in pixels

#define DISPLAY_MODE_CLEAR      0
#define DISPLAY_MODE_START      1
#define DISPLAY_MODE_AMPS       2
#define DISPLAY_MODE_TRIP       3
#define DISPLAY_MODE_STATS      4
#define DISPLAY_MODE_TEMPS      5
#define DISPLAY_MODE_CHARGE     6
#define DISPLAY_MODE_CONNECT    7

//void draw_display1 (float amps, bool mode_sport);
//void draw_display2 (int temp_batt1, int temp_batt2, int temp_cont, int temp_motor, bool warn_batt, bool warn_cont);
void draw_display (Adafruit_SSD1306, uint8_t, VOTOL_Response_t*, BMS_SOC_Report_t*, BMS_Temp_Report_t*, BMS_Fault_Report_t*, Trip_stats_t*);

extern Adafruit_SSD1306 display1;
extern Adafruit_SSD1306 display2;

#endif
