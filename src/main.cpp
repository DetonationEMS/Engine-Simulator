/********************************************************************************************
* Standalone Engine Simulator
* Aaron S. (Detonation) 2023
******************************
  DESCRIPTION
  ====================
  Simple Engine Crank/Cam Trigger Simulator

  This project is a personal learning experience and isn't for end-user use.
  It will contain bugs, efficiency issues and is generally incomplete.
  If you are looking for an engine simulator I recommend the Speeduino fork of Ardustim.
       https://github.com/speeduino/Ardu-Stim
********************************************************************************************/

/********************************************************************************************
Notes: Nano328 is the only properly working board at the moment.

 To-do:
 Figure out a way to scale pattern outputs
 Get more MCU's working. Would like esp32 or equivalently powerful MCUs.
    Less power MCU's like Attiny, Nano168 aren't priority and may be later abandoned.
********************************************************************************************/

#include <Arduino.h>
#include "main.h"

// Doesn't work properly yet.
#if defined(ESP32)
#include "board_esp32.h"
#include "display.h"
#include <pgmspace.h>
#endif

// Works well
#if defined(AVR328)
#include "board_avr328.h"
#include "display.h"
#include <avr/pgmspace.h>
#endif

#if defined(PICO)
#include "board_pico.h"
#include "display.h"
#include <avr/pgmspace.h>

#endif

uint16_t startTime;

void setup()
{
  initBoard(); // Initialize Board
}

void loop()
{
  // Delay for initBoard
  if (millis() - startTime < 250)
  {
    return; // Delay is not over, return without executing the rest of the loop
  }

  // Most of the work is done by timers/interrupts.
  // Calling adc() is for the pot RPM input control.

  adc();          // RPM control loop. Pot is read in ISR(ACD_vect)
  patternCheck(); // Various instuction needed each time a new pattern is selected.

#if defined(PICO)
  output();
#endif
}