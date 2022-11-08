#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>
#include "pins.h"

//#define DEBUG

#ifdef DEBUG
  extern HardwareSerial DebugSerial;
#endif

#endif
