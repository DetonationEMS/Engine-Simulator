#ifndef BOARD_ESP32_H
#define BOARD_ESP32_H
#if defined(ESP32)
#include <Arduino.h>

// Define constants for RPM and Array Selection control
#define tmpRPM_Shift 4 // x16, 0-16384 RPM via pot
#define maxRPM 16384   // MAX RPM via pot control
#define minWheels 1

void encoder();
void patternCheck();
void initBoard();
void adc();
void output();

#endif
#endif
