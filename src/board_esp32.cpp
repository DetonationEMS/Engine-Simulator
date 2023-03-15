#if defined(ESP32DEV)
#include "board_esp32.h"
#include <stdint.h>
#include <EEPROM.h>

#include "trigger_arrays.h"
#include "structures.h"

extern wheels Wheels[];

uint8_t rpmPot = 36;  // Pin numbers are defined in board files.
uint8_t crankPin = 12; // Pin for crank signal output
uint8_t camPin = 13;   // Pin for camshaft signal output

bool looping = true; // Define the analog input pin

uint32_t timerCount;    // Store value used to compare to interval
uint8_t currentPattern; // store the currently selected patter.
uint8_t lastPattern;    // Store previous selected output pattern
uint16_t patternLength; // Store value of selected pattern size
uint16_t currentIndex = 11;  // Store current pattern indexed value.

uint16_t sensorValue; // Store variable from potentiometer
uint32_t interval;    // Store mapped value used to compare to timerCount

void initBoard()
{
    pinMode(rpmPot, INPUT); // set potentiometer as an input
    
    pinMode(crankPin, OUTPUT); // set crankPin as an output
    pinMode(camPin, OUTPUT);   // set camPin as an output
}


// Pattern output really needs to be removed from the loop.
void esp32_loop()
{
    if (looping == true)
    {
        int pinValue = pgm_read_byte(&Wheels[currentPattern].selectedPattern[currentIndex]);
        digitalWrite(crankPin, pinValue & 0x01);
        digitalWrite(camPin, (pinValue >> 1) & 0x01);

        timerCount++; // increment the timer count
        if (timerCount >= interval)
        {
            if (++currentIndex == Wheels[currentPattern].patternLength)
            {                     // walk the pattern
                currentIndex = 0; // when the end of the pattern is reached go back to the start.
            }
            timerCount = 0;
        }
    }
}

#endif