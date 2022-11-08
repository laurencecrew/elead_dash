/* 
  timers.h
  timer related headers
*/

#ifndef TIMERS_H_INCLUDED
#define TIMERS_H_INCLUDED

#include <Arduino.h>

// function pointer for timer calls
typedef void (* TmrCallback) ();

// function headers
void TMR_Init ();
void TMR_SetInterval (TmrCallback, word);
void TMR_ClearInterval ();
void TMR_Resume ();

#endif
