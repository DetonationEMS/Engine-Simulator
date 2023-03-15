#ifndef STRUCTURES_H
#define STRUCTURES_H

#if defined(AVR328)
#include <avr/pgmspace.h>
#endif

#include <stdint.h>
#include "trigger_arrays.h"

typedef struct _wheels wheels;
struct _wheels
{
  const char *patternName PROGMEM;              // "Friendly Name"
  const unsigned char *selectedPattern PROGMEM; // store selected trigger pattern.
  const float rpm_scaler;                       // store Ardustim RPM scaler.
  const uint16_t patternLength;                 // store pattern length.
  const uint16_t wheel_degrees;                 // store pattern in degrees of rotation.
};

#endif