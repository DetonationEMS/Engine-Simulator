#ifndef BOARD_PICO_H
#define BOARD_PICO_H
/*
Pico Notes(apply to me personally):
May be able to use some tricks from the ESP32 attempt
Pico notes in binder #3
*/

// Define constants for RPM and Array Selection control
#define maxRPM 16384   // MAX RPM via pot control
#define minWheels 1

#include <Arduino.h>

void updateEncoder();
void patternCheck();
void initBoard();
void output();
uint8_t get_bitshift_from_prescaler(uint8_t *prescalerBits);
void get_prescaler_bits(uint32_t *potential_oc_value, uint8_t *prescaler, uint8_t *bitshift);
void calculate_delay_us(uint32_t new_rpm);
void adc();


#endif