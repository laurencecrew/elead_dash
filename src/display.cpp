#include "display.h"
#include "pins.h"
#include "debug.h"

Adafruit_SSD1306 display1 (SCREEN_WIDTH, SCREEN_HEIGHT, COLUMN_OFFSET, &SPI, OLED_DC_1, OLED_RESET_1, OLED_CS_1);
Adafruit_SSD1306 display2 (SCREEN_WIDTH, SCREEN_HEIGHT, COLUMN_OFFSET, &SPI, OLED_DC_2, OLED_RESET_2, OLED_CS_2);

void draw_display (Adafruit_SSD1306 display, uint8_t mode, VOTOL_Response_t *resp, BMS_SOC_Report_t *soc_1, BMS_Temp_Report_t *temp_1, BMS_Fault_Report_t *fault_1, Trip_stats_t *trip_stats)
{
  int16_t  x1, y1;
  uint16_t w, h;
  char str1[5], str2[5];
  int16_t amps;
  uint16_t volts;
  int16_t temp_batt1, temp_batt2, temp_cont, temp_motor;

  display.clearDisplay();

  switch (mode)
  {
    case DISPLAY_MODE_CLEAR:

    break;

    case DISPLAY_MODE_START:

      // screen label
       display.drawBitmap (23, 15, bm_icon_power, 18, 20, SSD1306_WHITE);

    break;

    case DISPLAY_MODE_AMPS:
      
      // screen label
      display.drawBitmap (49, 3, bm_label_amps, 12, 13, SSD1306_WHITE);

      // draw amps value
      if (resp != NULL)
      {
        // mode icon
        if (VOTOL_get_sport_mode (resp))
          display.drawBitmap (3, 3, bm_icon_sport, 37, 13, SSD1306_WHITE);

        amps = VOTOL_get_amps (resp);

        if ((amps < 100) && (amps > -100)) // 100 because amps value is * 10
          sprintf (str1, "%d.%d", amps / 10, abs(amps % 10)); // show low values with decimal point
        else
          sprintf (str1, "%d", amps / 10); // show higher values as whole numbers
      }
      else
      {
          strcpy (str1, "---  ");
      }

      display.setFont(&FreeSansBold18pt7b);
      display.setTextSize(1);             // Normal 1:1 pixel scale
      display.setTextColor(SSD1306_WHITE);

      display.getTextBounds(str1, 0, 0, &x1, &y1, &w, &h);
      display.setCursor((SCREEN_WIDTH - w) - 3, SCREEN_HEIGHT - 3); // Right align with 2px R, 3px B borders
      display.print(str1);

      #ifdef DEBUG
        //DebugSerial.print ("Display amps: ");
        //DebugSerial.println (str);
      #endif

    break;

    case DISPLAY_MODE_TRIP:

    break;

    case DISPLAY_MODE_STATS:

    break;

    case DISPLAY_MODE_TEMPS:

      // screen label
      display.drawBitmap (44, 3, bm_label_degrees, 15, 9, SSD1306_WHITE);

      // temperature icons
      display.drawBitmap (3, 18, bm_icon_batt1, 6, 11, SSD1306_WHITE);
      //display.drawBitmap (34, 18, bm_icon_batt2, 6, 11, SSD1306_WHITE);
      display.drawBitmap (3, 34, bm_icon_cont, 6, 11, SSD1306_WHITE);
      display.drawBitmap (34, 34, bm_icon_motor, 6, 11, SSD1306_WHITE);

      // temperature values
      display.setFont(&FreeSansBold9pt7b);
      display.setTextSize(1);              // 1:1 pixel scale
      display.setTextColor(SSD1306_WHITE);

      // BMS warnings
      if (fault_1 != NULL)
      {
        // warning icons
        if (BMS_get_fault(fault_1))
          display.drawBitmap (3, 3, bm_warn_batt, 6, 10, SSD1306_WHITE);
      }

      // BMS temperatures
      if (temp_1 != NULL)
      {
        temp_batt1 = BMS_get_temp(temp_1);
        //temp_batt2 = BMS_get_temp(temp_2);

        sprintf (str1, "%d", temp_batt1);
        //sprintf (str2, "%d", temp_batt2);

      }
      else
      {
          strcpy (str1, " --");
          //strcpy (str2, " --");
      }
 
      // batt 1 temp
      display.setCursor(10, 28);
      display.print(str1);

      // batt 2 temp
      //display.setCursor(41, 28);
      //display.print(str2);

      // VOTOL temperatures
      if (resp != NULL)
      {
        if (VOTOL_get_fault (resp))
          display.drawBitmap (11, 3, bm_warn_cont, 6, 10, SSD1306_WHITE);

        temp_cont = VOTOL_get_contr_temp(resp);
        temp_motor = VOTOL_get_motor_temp(resp);

        sprintf (str1, "%d", temp_cont);
        sprintf (str2, "%d", temp_motor);

      }
      else
      {
          strcpy (str1, " --");
          strcpy (str2, " --");
      }
 
      // controller temp
      display.setCursor(10, 44);
      display.print(str1);

      // motor temp
      display.setCursor(41, 44);
      display.print(str2);
      
    break;

    case DISPLAY_MODE_CHARGE:

      if (soc_1 != NULL)
      {
        volts = BMS_get_volts (soc_1);
        amps = BMS_get_amps (soc_1) * 2; // Assume current is equal between packs. TODO: read soc_2 and add once CANBus sorted out

        // screen label
        display.drawBitmap (3, 3, bm_icon_charge, 40, 13, SSD1306_WHITE);

        display.setFont(&FreeSansBold9pt7b);
        display.setTextSize(1);              // 1:1 pixel scale
        display.setTextColor(SSD1306_WHITE);

        sprintf (str1, "%d.%dV", volts / 10, volts % 10);
        sprintf (str2, "%d.%dA", amps / 10, abs (amps % 10));
      }
      else
      {
          strcpy (str1, " --");
          strcpy (str2, " --");
      }

      // charge voltage
      display.setCursor(3, 31);
      display.print(str1);

      // charge current
      display.setCursor(3, 46);
      display.print(str2);

    break;

    case DISPLAY_MODE_CONNECT:

      // screen label
       display.drawBitmap (22, 14, bm_icon_connect, 19, 19, SSD1306_WHITE);

    break;
  }

  // go!
  display.display();
}

