// DOES NOT WORK. Threw together to progress the idea in the future.
#if defined(PICO)
#include <Arduino.h>

// #include <pico/stdlib.h>
#include "pico.h"
#include <cstdint>
#include <stdio.h>
#include <hardware/adc.h>
#include <hardware/gpio.h>

#include "board_avr328.h"
#include "wheel_defs.h"
#include "structures.h"
#include "display.h"

// Rotary Encoder pins
#define picoEncoderPinA 16
#define picoEncoderPinB 17

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

void encoder(uint gpio, uint32_t events)
{
  triggerOutput = false; // Stop the loop
  currentIndex = 0;      // Reset the array index to 0

  // Read the current state of the encoder's two digital pins
  uint8_t a = gpio_get(picoEncoderPinA);
  uint8_t b = gpio_get(picoEncoderPinB);

  bool lastEncoded = 0;
  // Update currentPattern based on the change in encoder value
  if (lastEncoded != a && a == b)
  {
    // Clockwise rotation
    encoderValue++;
    currentPattern = static_cast<WheelType>((currentPattern + 1) % (MAX_WHEELS));
    updateDisplayName = true;
  }
  else if (lastEncoded != a && a != b)
  {
    // Counter-clockwise rotation
    encoderValue--;
    currentPattern = static_cast<WheelType>((currentPattern + 1) % (MAX_WHEELS));
    updateDisplayName = true;
  }
  lastEncoded = a;

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

  // stdio_init_all();

  // Initialize Pins for rotary encoder
  // Set pins as inputs with pull-ups
  _gpio_init(picoEncoderPinA);
  gpio_set_dir(picoEncoderPinA, GPIO_IN);
  gpio_pull_up(picoEncoderPinA);
  gpio_set_irq_enabled_with_callback(picoEncoderPinA, GPIO_IRQ_EDGE_FALL, true, (gpio_irq_callback_t)encoder);

  _gpio_init(picoEncoderPinB);
  gpio_set_dir(picoEncoderPinB, GPIO_IN);
  gpio_pull_up(picoEncoderPinB);
  gpio_set_irq_enabled_with_callback(picoEncoderPinB, GPIO_IRQ_EDGE_FALL, true, (gpio_irq_callback_t)encoder);

  // Check if the loaded value is within the range.
  // if (currentPattern < minWheels || currentPattern > MAX_WHEELS)
  //{
  // currentPattern = 42; // Audi
  // currentPattern = 11; // 12-1
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

  if (triggerOutput == true)
  {
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
}

#endif
