#include "gauge.h"
#include "debug.h"

TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(GAUGE_PWM), PinMap_PWM);
uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(GAUGE_PWM), PinMap_PWM));

// Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
HardwareTimer *GaugeTimer = new HardwareTimer(Instance); // for gauge PWM output

void GAUGE_Init()
{
  // Gauge PWM output  
  GaugeTimer->setPWM(channel, GAUGE_PWM, PWM_FREQ, GAUGE_MID);
}

// set the gauge from 0-100
void GAUGE_Set (uint8_t val)
{
  // look up the PWM value for the given gauge value.
  GaugeTimer->setCaptureCompare(channel, getCompare(val), TICK_COMPARE_FORMAT); // PERCENT_COMPARE_FORMAT);
  GaugeTimer->resume();

    #ifdef DEBUG
      DebugSerial.printf ("Gauge val: %d\n", val);
      //DebugSerial.printf ("Compare: %d\n", getCompare(val));
    #endif
}

uint32_t getCompare (uint8_t val)
{
    return ((uint32_t)gauge_lookup[val]);
}
