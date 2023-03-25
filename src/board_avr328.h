#ifndef BOARD_AVR328_H
#define BOARD_AVR328_H

#include <Arduino.h>
#include <avr/pgmspace.h>

// Define constants for RPM and Array Selection control
#define tmpRPM_Shift 4 // x16, 0-16384 RPM via pot
#define maxRPM 16384   // MAX RPM via pot control
#define minWheels 1

// Variables for analog port
// extern uint8_t analogPort; // Store analog port to be being used
// extern uint16_t adc0;      // Store value of RPM potentiometer
// extern bool adc0_Ready;    // Flag used to manipulate the port

// Variables for desired RPM
// extern uint16_t desiredRPM; // Define a variable to store the desired RPM value
// extern uint16_t tempRPM;    // Store variable tempRPM

// Variables for output interrupt
extern bool triggerOutput; // Store value used for output interrupt

// Variables for timer control
extern bool resetPrescaler;   // Flag used to reset timer prescaler
extern uint8_t prescalerBits; // Store the prescaler bits
extern uint16_t new_OCR1A;    // New value of OCR1A for timer control

// Variables for pattern selection
extern uint8_t currentPattern; // Store the currently selected pattern.
extern uint16_t currentIndex;  // Store the current pattern indexed value.

// Define a function to initialize board hardware
void initBoard();
void adc();
void reset_new_OCR1A(uint32_t new_rpm);
void patternCheck();

// Button not really needed
// void button();

// Enumerations for pre-scaler
enum
{
    PRESCALE_1 = 1,
    PRESCALE_8 = 2,
    PRESCALE_64 = 3,
    PRESCALE_256 = 4,
    PRESCALE_1024 = 5
};

#endif
