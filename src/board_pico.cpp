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

// TIMER1 Compare
bool resetPrescaler = false; // Flag used to reset timer pre-scaler
uint8_t prescalerBits = 0;   // Store the pre-scaler bits
uint16_t new_OCR1A;          // New value of OCR1A for timer control

// Rotary Encoder
uint8_t lastEncoded = 0;
uint16_t encoderValue = 0;

// Variables for analog port
uint8_t analogPort; // Store analog port to be being used
uint16_t adc0;      // Store value of RPM potentiometer

// Variables for desired RPM
uint16_t desiredRPM = 0; // Define a variable to store the desired RPM value
float tempRPM = 0;       // Store variable tempRPM

// Pattern selection
extern wheels Wheels[];
uint8_t currentPattern = 11; // Store Currently selected pattern. Stored in EEPROM.
uint16_t currentIndex = 0;   // Store currentPattern's indexed value. This value must be set to zero each time the pattern changes, starts or stops.

// Flag for updating the display
bool updateDisplayName = true;

void rotary_encoder_isr()
{
  // Read the current state of the encoder's two digital pins
  uint32_t MSB = gpio_get(encoderPinA);
  uint32_t LSB = gpio_get(encoderPinB);

  // Combine the two bits into a single byte using bitwise operators
  uint32_t encoded = (MSB << 1) | LSB;

  // Update encoderValue based on the change in encoder value
  static uint32_t lastEncoded = 0;
  if (encoded != lastEncoded)
  {
    if ((lastEncoded == 0b00 && encoded == 0b01) || (lastEncoded == 0b01 && encoded == 0b11) || (lastEncoded == 0b11 && encoded == 0b10) || (lastEncoded == 0b10 && encoded == 0b00))
    {
      encoderValue++;
    }
    else if ((lastEncoded == 0b00 && encoded == 0b10) || (lastEncoded == 0b10 && encoded == 0b11) || (lastEncoded == 0b11 && encoded == 0b01) || (lastEncoded == 0b01 && encoded == 0b00))
    {
      encoderValue--;
    }
    lastEncoded = encoded; // Update lastEncoded to match encoded
  }
}

void updateEncoder()
{
  triggerOutput = false; // Stop the loop
  ISR_loop = false;      // Ready restart count flag

  currentIndex = 0; // Reset the array index to 0

  // Read the current state of the encoder's two digital pins
  uint8_t MSB = digitalRead(encoderPinA);
  uint8_t LSB = digitalRead(encoderPinB);

  // Combine the two bits into a single byte using bitwise operators
  uint8_t encoded = (MSB << 1) | LSB;

  // Update currentPattern based on the change in encoder value
  if (encoded != lastEncoded)
  {
    if ((lastEncoded == 0b00 && encoded == 0b01) || (lastEncoded == 0b01 && encoded == 0b11) || (lastEncoded == 0b11 && encoded == 0b10) || (lastEncoded == 0b10 && encoded == 0b00))
    {
      encoderValue++;
      if (encoderValue % 2 == 0)
      {
        currentPattern = static_cast<WheelType>((currentPattern + 1) % (MAX_WHEELS));
        updateDisplayName = true;
      }
    }
    else if ((lastEncoded == 0b00 && encoded == 0b10) || (lastEncoded == 0b10 && encoded == 0b11) || (lastEncoded == 0b11 && encoded == 0b01) || (lastEncoded == 0b01 && encoded == 0b00))
    {
      encoderValue--;
      if (encoderValue % 2 == 0)
      {
        currentPattern = static_cast<WheelType>((currentPattern - 1 + (MAX_WHEELS)) % (MAX_WHEELS));
        updateDisplayName = true;
      }
    }
  }
  lastEncoded = encoded; // Update lastEncoded to match encoded
  // EEPROM.put(0, currentPattern); // Store currentPattern to EEPROM

  ISR_loop = true; // Restart the loop
}

// Holds various instructions needed each time the pattern is changed
void patternCheck()
{
  // Checks flag to see if display needs to be updated (has to be better way to do this)
  if (updateDisplayName == true) // Called if new pattern needs to be displayed.
  {
    updateDisplayName = false;
    // updateDisplay(); // Calls function that updates display to new pattern
  }

  // Adds delay when triggerOutput changes from false to true. (Needed to prevent display and/or output from locking up)
  if (ISR_loop && loopStartTime == 0) // If ISR_loop has just changed from false to true
  {
    loopStartTime = millis(); // Store the start time of the delay
  }
  // If restartOutputTime has passed since the start time and the delay has not been reset
  uint16_t restartOutputTime = 1000;
  if (millis() - loopStartTime >= restartOutputTime && loopStartTime != 0)
  {
    triggerOutput = true; // Set output to true
    loopStartTime = 0;    // Reset the start time of the delay
  }
}

void initBoard()
{
  adc_init();
  adc_select_input(rpmPot);

  // COMMENTED OUT FOR DEBUG
  // EEPROM.get(0, currentPattern); // Get previously stored pattern

  // Check if the loaded value is within the range.
  // if (currentPattern < minWheels || currentPattern > MAX_WHEELS)
  //{
  currentPattern = 11; // If not use a default value of 11
  //}
}

void output()
{
  static uint32_t lastUpdateTime = 0;
  uint32_t currentTime = millis();

  if (triggerOutput == true)
  {
    if (currentTime - lastUpdateTime >= delay_us)
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
}

// This function sets new OCR1A value based on target RPM
void calculate_delay_us(uint32_t new_rpm)
{
  // Store temporary variables
  uint32_t tmp;

  // Calculate new OCR1A value based on current pattern and target RPM
  tmp = (uint32_t)(8000.0 / (Wheels[currentPattern].rpm_scaler * (float)(new_rpm < 10 ? 10 : new_rpm)));

  resetPrescaler = true; // Set resetPrescaler flag to true
}

void adc()
{

  uint16_t pot_value = adc_read(); // this read working

  // float new_rpm = 100 + ((16000 - 100) * (pot_value / 65535.0));
  float new_rpm = 100 + ((16000 - 100) * (pot_value / 65535.0));

  //uint16_t tempRPM = (uint32_t)(((16000 - 100) * (pot_value / 65535.0) / (Wheels[currentPattern].rpm_scaler *

  // Check if tempRPM is greater than maxRPM
  // If it is, assign the value of maxRPM to tempRPM, else keep it as is
  tempRPM = (tempRPM > maxRPM) ? maxRPM : tempRPM;
  delay_us = tempRPM;        // Assign the value of tempRPM to the desiredRPM variable
  
  // calculate_delay_us(tempRPM); // Call the reset_new_OCR1A function with tempRPM as argument

  // If resetPrescaler flag is true, reset pre-scaler based on pre-scalerBits value
  if (resetPrescaler)
  {
    resetPrescaler = false; // Reset resetPrescaler flag
  }
}
#endif
