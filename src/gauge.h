#ifndef GAUGE_H
#define GAUGE_H

#include <Arduino.h>
#include "pins.h"

// TODO: Calibration / lookup table
#define PWM_FREQ  50000 // Hz

// starting values - percentage duty cycle
#define GAUGE_HIGH 25
#define GAUGE_MID 44
#define GAUGE_LOW 75

void GAUGE_Init ();
void GAUGE_Set (uint8_t);

// internal - need to declare?
uint32_t getCompare (uint8_t);

const uint16_t gauge_lookup[] = {
    1016, 1006, 996, 986, 977, 967, 957, 948, 938, 929,
    920, 911, 902, 893, 884, 875, 866, 857, 849, 840,
    832, 823, 815, 807, 798, 790, 782, 774, 766, 759,
    751, 743, 736, 728, 721, 714, 706, 699, 692, 685,
    678, 671, 665, 658, 651, 645, 638, 632, 626, 619,
    613, 607, 601, 595, 589, 584, 578, 572, 567, 561,
    556, 551, 546, 540, 535, 530, 525, 521, 516, 511,
    507, 502, 498, 493, 489, 485, 481, 477, 473, 469,
    465, 461, 458, 454, 450, 447, 444, 440, 437, 434,
    431, 428, 425, 422, 420, 417, 415, 412, 410, 407, 405
};

#endif

/* Calibration values

395 100
499 75
630 50
755 25
812 20
1030    0

*/