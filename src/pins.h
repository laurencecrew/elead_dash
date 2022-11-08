#ifndef PINS_H
#define PINS_H

#define DEBUG_USART     USART1  // TX1: PA9 RX1: PA10
#define VOTOL_USART     USART3  // TX3: PB10 RX3: PB11

#define CAN_REMAP        2   // CAN_RX mapped to PB8, CAN_TX mapped to PB9 (not available on 36-pin package)

#define OLED_DC_1       PB6
#define OLED_CS_1       PB5 
#define OLED_RESET_1    PB7

#define OLED_DC_2       PA4
#define OLED_CS_2       PA2  
#define OLED_RESET_2    PA3

#define LIGHTS_IN       PB12
#define TURN_L_IN       PA12
#define TURN_R_IN       PA11
#define BRAKE_IN        PB13

#define MODE_1_IN       PA8
#define MODE_2_IN       PA15

#define TURN_L_OUT      PB1
#define TURN_R_OUT      PB0
#define TURN_IND_OUT    PB3
#define GAUGE_PWM       PB14

#endif

