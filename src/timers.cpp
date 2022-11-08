/* 
  timers.c
  timer related functions and globals
*/

#include "timers.h"

/* globals */

HardwareTimer *MyTim;

/* functions */

// initialise the interval timer
void TMR_Init()
{
  //#if defined(TIM1)
  //  TIM_TypeDef *Instance = TIM1;
  //#else
    TIM_TypeDef *Instance = TIM2; // Force TIM2 as TIM1 is used by PWM
  //#endif
  
  MyTim = new HardwareTimer (Instance);
  MyTim->setMode (2, TIMER_DISABLED);
}

// set a function for interval callback using Tick timer
// inputs:  f = function pointer to interval function;
//      t = time in ms; up to 65535ms
void TMR_SetInterval (TmrCallback f, word t)
{
  // Set a compare interrupt on Timer0 - 1ms intervals
  MyTim->setOverflow (t * 1000, MICROSEC_FORMAT);
  MyTim->attachInterrupt (f);
  MyTim->resume(); // start the ms timer
}

// clear the interval callback
void TMR_ClearInterval ()
{
  MyTim->pause(); // start the ms timer
  MyTim->detachInterrupt();
}

void TMR_Resume()
{
  MyTim->resume(); // start the ms timer
}
