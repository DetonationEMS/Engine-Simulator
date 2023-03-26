#ifndef STANDALONE_DISPLAY_H
#define STANDALONE_DISPLAY_H

#include <Arduino.h>
#include <avr/pgmspace.h>

void sendCommand(uint8_t command);
void sendData(const uint8_t* data, uint16_t size);
void initOled();
void clearDisplay();
void setPixel(uint8_t x, uint8_t y, bool value);
int main();

#endif // STANDALONE_DISPLAY_H