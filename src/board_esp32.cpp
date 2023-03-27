/*
 *  ESP32 doesn't work properly.
 *  Testing done on ESP32-S2. ( https://www.amazon.com/dp/B0B7WY3MWG?psc=1&ref=ppx_yo2ov_dt_b_product_details )
 *
 * Although OCR1A doesn't exist on the ESP32 platform the variable name is retained
 * for my ease of understanding while coding. Variable names will be changed appropriately as I
 * better understand the ESP32 onboard timers.
 *
 * I am learning the ACD input of the esp32 might not be as friendly as I would like for this project.
 * I need to do more research and possible switch to a different MCU.
 *
 */
#if defined(ESP32)
#include <Arduino.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <EEPROM.h>

#include "board_esp32.h"
#include "wheel_defs.h"
#include "structures.h"
// #include "display.h"

// Rotary Encoder pins
#define ESP32encoderPinA GPIO_NUM_36
#define ESP32encoderPinB GPIO_NUM_38

// Output Pins
#define ESP32rpmPot GPIO_NUM_12
#define ESP32crankPin GPIO_NUM_3
#define ESP32camPin GPIO_NUM_5

// Non-Pico Specific Variables.
uint16_t loopStartTime = 0; // Store the start time of the delay for output restart
bool ISR_loop = 1;          // Flag that delays output restart
bool triggerOutput = true;  // Store value used for output interrupt
float triggerDelay;         // Store delay maths
uint16_t prescaler;
float scaledRPM;
volatile unsigned long pulseDuration = 0;
volatile bool outputState = LOW;
unsigned long previousMillis = 0;
volatile unsigned long lastPulseDuration = 0;

// Rotary Encoder
uint8_t lastEncoded = 0;
uint16_t encoderValue = 0;

// Variables for analog input and RPM control
uint16_t potValue;
uint16_t desiredRPM = 0; // Define a variable to store the desired RPM value

// Pattern selection
extern wheels Wheels[];
uint8_t currentPattern = 11;   // Store Currently selected pattern. Stored in EEPROM.
uint16_t currentIndex = 0;     // Store currentPattern's indexed value. This value must be set to zero each time the pattern changes, starts or stops.
bool updateDisplayName = true; // Flag for updating the display

// Configure the hardware timer for RPM control
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Define the timer interrupt service routine
void IRAM_ATTR onTimer()
{
  if (triggerOutput == true)
  {
    int pinValue = pgm_read_byte(&Wheels[currentPattern].selectedPattern[currentIndex]);
    digitalWrite(ESP32crankPin, pinValue & 0x01);
    digitalWrite(ESP32camPin, (pinValue >> 1) & 0x01);

    if (++currentIndex == Wheels[currentPattern].patternLength)
    {
      currentIndex = 0;
    }
  }
}

void encoder()
{
  triggerOutput = false; // Stop the loop
  ISR_loop = false;      // Ready restart count flag

  currentIndex = 0; // Reset the array index to 0

  // Read the current state of the encoder's two digital pins
  uint8_t MSB = digitalRead(ESP32encoderPinA);
  uint8_t LSB = digitalRead(ESP32encoderPinB);

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

  // EEPROM.writeInt(0, currentPattern); // Store currentPattern to EEPROM

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

// Define a function to initialize board hardware
void initBoard()
{
  // Initialize and configure the hardware timer
  timer = timerBegin(0, 80, true);          // Timer 0, 80 divider (1MHz), count-up
  timerAttachInterrupt(timer, &onTimer, true);     // Attach onTimer() function as interrupt
  timerAlarmWrite(timer, pulseDuration / 2, true); // Set initial alarm value and auto-reload
  timerAlarmEnable(timer);                         // Enable the timer alarm

  // Configure pin modes for inputs and outputs
  pinMode(ESP32crankPin, OUTPUT); // set crankPin as an output
  pinMode(ESP32camPin, OUTPUT);   // set camPin as an output

  pinMode(ESP32rpmPot, INPUT); // set potentiometer as an input
  analogReadResolution(10);

  pinMode(ESP32encoderPinA, INPUT_PULLUP);
  pinMode(ESP32encoderPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ESP32encoderPinA), encoder, CHANGE); // Interrupts for rotary encoder
  attachInterrupt(digitalPinToInterrupt(ESP32encoderPinB), encoder, CHANGE); // Interrupts for rotary encoder

  // Check if the loaded value is within the range.
  if (currentPattern < minWheels || currentPattern > MAX_WHEELS)
  {
    currentPattern = 11; // If not use a default value of 11
  }
}

void adc()
{
  unsigned long currentMillis = micros();
  if (currentMillis - previousMillis >= 10)
  {
    previousMillis = currentMillis;

    potValue = analogRead(ESP32rpmPot);
    // Map the potentiometer value directly to the RPM range
    scaledRPM = map(potValue, 0, 1023, minRPM, maxRPM);
    pulseDuration = 60 * 1000000 / scaledRPM;

    // Check if pulseDuration has changed, and update the timer alarm value if necessary
    if (pulseDuration != lastPulseDuration)
    {
      timerAlarmWrite(timer, pulseDuration / 2, true);
      lastPulseDuration = pulseDuration;
    }
  }
}

void output()
{
  // Not used at this time
}
#endif