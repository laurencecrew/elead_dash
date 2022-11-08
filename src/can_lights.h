#ifndef CAN_LIGHTS_H
#define CAN_LIGHTS_H

#include <Arduino.h>

#define LIGHTS_CAN_ID    0x110 // 11-bit standard ID, <= 0x800
#define LIGHTS_DATA_LEN  1

typedef union
{
  struct
  {
    uint8_t tail:1;
    uint8_t brake:1;
    uint8_t turn_l:1;
    uint8_t turn_r:1;
  } bits;

  uint8_t data;

} LIGHTS_Data_t;

void LIGHTS_send_data (LIGHTS_Data_t*);

#endif
