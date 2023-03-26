#ifndef BOARD_PICO_H
#define BOARD_PICO_H
/*
Pico Notes(apply to me personally):
May be able to use some tricks from the ESP32 attempt
Pico notes in binder #3
*/

#include <Arduino.h>

void encoder(uint gpio, uint32_t events);
void patternCheck();
void initBoard();
void encoder();
void output();
void adc();


#endif