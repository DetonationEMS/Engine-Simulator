/********************************************************************************************
* Standalone Engine Simulator
* Aaron S. (Detonation) 2023
******************************
  DESCRIPTION
  ====================
  Simple Engine Crank/Cam Trigger Simulator

  This project is a personal learning experience and isn't for end-user use.
  It will constain bugs, efficiancy issues and is generally incomplete.
  If you are looking for an engine simulator I recommend the Speeduino fork of Ardustim.
       https://github.com/speeduino/Ardu-Stim
********************************************************************************************/

/********************************************************************************************
Notes: Nano328 is the only properly working board at the moment.

 To-do:
 Serial Debug
 Add pattern selection
 Figure out a way to scale pattern outputs
 Add a Screen for board selection. (0.96 OLED)
 Get more MCU's working. Would like esp32 or equivalently powerful MCUs.
    Less power MCU's like Attiny, Nano168 aren't priority and may be later abandoned.
********************************************************************************************/

#include <Arduino.h>
#include "main.h"

// Doesn't work properly. Need to learn to use esp32 timers
#if defined(ESP32DEV)
#include "board_esp32.h"
#endif

// Works well
#if defined(AVR328)
#include "board_avr328.h"
#include "display.h"
#include <avr/pgmspace.h>
#endif

uint16_t startTime;

void setup()
{
#if defined(USE_TINY4K)
  loadDisplay();
#endif

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
#if defined(AVR328)
  adc();  // RPM control loop. Pot is read in ISR(ACD_vect)
  patternCheck(); // Various instuction needed each timer a new pattern is selected. 
#endif
}