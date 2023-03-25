#ifndef STRUCTURES_H
#define STRUCTURES_H

#if defined(AVR328)
#include <avr/pgmspace.h>
#endif

#include <stdint.h>
#include "wheel_defs.h"

// Important constants for RPM and Array Selection control
#define tmpRPM_Shift 4 // x16, 0-16384 RPM via pot
#define minRPM 100
#define maxRPM 16384   // MAX RPM via pot control
#define minWheels 1

// Access the specific pattern stored in Wheels using currentPattern as an index
// const char *selectedPatternName = Wheels[currentPattern].patternName;
// const unsigned char *selectedTriggerPattern = Wheels[currentPattern].selectedPattern;
// const float rpmScaler = Wheels[currentPattern].rpm_scaler;
// const uint16_t patternLength = Wheels[currentPattern].patternLength;
// const uint16_t wheelDegrees = Wheels[currentPattern].wheel_degrees;
// Use the selected trigger pattern, RPM scaler, pattern length, and wheel degrees as needed

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