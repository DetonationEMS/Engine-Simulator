// DOES NOT WORK. Threw together to progress the idea in the future.
#if defined(PICO)
#include <Arduino.h>
#include <cstdint>
#include <stdio.h>
#include <hardware/adc.h>
#include <hardware/gpio.h>

#include "board_avr328.h"
#include "wheel_defs.h"
#include "structures.h"
#include "display.h"

// Rotary Encoder pins
#define encoderPinA 19
#define encoderPinB 20

// Output Pins
#define rpmPot ADC0
#define crankPin 8
#define camPin 9

// TIMER1 Interrupt Service Routine (ISR)
uint16_t loopStartTime = 0; // Store the start time of the delay for output restart
bool ISR_loop = 1;          // Flag that delays output restart
bool triggerOutput = true;  // Store value used for output interrupt
uint32_t delay_us;
uint32_t delay_ms;
float triggerDelay;

// TIMER1 Compare
bool resetPrescaler = false; // Flag used to reset timer pre-scaler
uint8_t prescalerBits = 0;   // Store the pre-scaler bits
uint16_t new_OCR1A;          // New value of OCR1A for timer control

// Rotary Encoder
uint8_t lastEncoded = 0;
uint16_t encoderValue = 0;

// Variables for analog port
uint16_t potValue;

// Variables for desired RPM
uint16_t desiredRPM = 0; // Define a variable to store the desired RPM value
float tempRPM = 0;       // Store variable tempRPM

// Pattern selection
extern wheels Wheels[];
uint8_t currentPattern = 11; // Store Currently selected pattern. Stored in EEPROM.
uint16_t currentIndex = 0;   // Store currentPattern's indexed value. This value must be set to zero each time the pattern changes, starts or stops.

// Flag for updating the display
bool updateDisplayName = true;

void updateEncoder()
{
  // empty while debug
}

// Holds various instructions needed each time the pattern is changed
void patternCheck()
{
  // empty while debug
}

void initBoard()
{
  adc_init();
  adc_select_input(rpmPot);

  // Check if the loaded value is within the range.
  // if (currentPattern < minWheels || currentPattern > MAX_WHEELS)
  //{
  // currentPattern = 42; // Audi
  currentPattern = 11; // 12-1
  // currentPattern = 7; // 24-1
  //}
}

void adc()
{
  potValue = adc_read(); // this read working

  // Normalize the potentiometer value to a range between 0 and 1000
  int normalizedPotValue = (potValue * 4095) / 4095;

  int desiredRPM = minRPM + (((maxRPM - minRPM) * normalizedPotValue) / 4096);

  // Access the specific pattern stored in Wheels using currentPattern as an index
  double scaler = Wheels[currentPattern].rpm_scaler * (Wheels[currentPattern].wheel_degrees / 360.0);

  triggerDelay = 600000.0 / ((double)desiredRPM * scaler);
}

void output()
{
  // Output Loop
  static uint32_t lastUpdateTime = 0;
  uint32_t currentTime = micros();

  // if (triggerOutput == true)
  //{
  if (currentTime - lastUpdateTime >= triggerDelay)
  {
    int pinValue = pgm_read_byte(&Wheels[currentPattern].selectedPattern[currentIndex]);
    digitalWrite(crankPin, pinValue & 0x01);
    digitalWrite(camPin, (pinValue >> 1) & 0x01);

    if (++currentIndex == Wheels[currentPattern].patternLength)
    {                   // walk the pattern
      currentIndex = 0; // when the end of the pattern is reached go back to the start.
    }

    lastUpdateTime = currentTime;
  }
}
//}

#endif
