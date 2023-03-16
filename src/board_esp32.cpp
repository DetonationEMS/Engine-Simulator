/*
 *  ESP32 isn't functional yet. Simply "ported" over from the nano.
 *  Testing done on ESP32-S2. ( https://www.amazon.com/dp/B0B7WY3MWG?psc=1&ref=ppx_yo2ov_dt_b_product_details )
 *  It's small and uses USB-C
 *  and a very old "dev kit"
 */

#if defined(ESP32)
#include <Arduino.h>
#include <stdint.h>
#include <EEPROM.h>
#include <driver/timer.h>
#include <esp_timer.h>

#include "board_esp32.h"
#include "trigger_arrays.h"
#include "structures.h"
// #include "display.h"

// Rotary Encoder pins
#define encoderPinA GPIO_NUM_18
#define encoderPinB GPIO_NUM_19

// Output Pins
#define rpmPot GPIO_NUM_4
#define crankPin GPIO_NUM_2
#define camPin GPIO_NUM_15

// TIMER1 Interrupt Service Routine (ISR)
uint16_t loopStartTime = 0; // Store the start time of the delay for output restart
bool ISR_loop = 1;          // Flag that delays output restart
bool triggerOutput = true;  // Store value used for output interrupt

// TIMER1 Compare
bool resetPrescaler = false; // Flag used to reset timer prescaler
uint8_t prescalerBits = 0;   // Store the prescaler bits
uint16_t new_OCR1A;          // New value of OCR1A for timer control

// Rotary Encoder
uint8_t lastEncoded = 0;
uint16_t encoderValue = 0;

// Variables for analog port
uint8_t analogPort; // Store analog port to be being used
uint16_t adc0;      // Store value of RPM potentiometer
bool adc0_Ready;    // Flag used to manipulate the port

// Variables for desired RPM
uint16_t desiredRPM = 0; // Define a variable to store the desired RPM value
uint16_t tempRPM = 0;    // Store variable tempRPM

// Pattern selection
extern wheels Wheels[];
uint8_t currentPattern = 11; // Store Currently selected pattern. Stored in EEPROM.
uint16_t currentIndex = 0;   // Store currentPattern's indexed value. This value must be set to zero each time the pattern changes, starts or stops.

// Flag for updating the display
bool updateDisplayName = true;

// Configure the hardware timer for RPM control
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Define the timer interrupt service routine
void IRAM_ATTR timerISR()
{
    portENTER_CRITICAL_ISR(&timerMux); // Ensure that no other interrupts execute during this routine
    if (triggerOutput == true)
    {
        int pinValue = pgm_read_byte(&Wheels[currentPattern].selectedPattern[currentIndex]);
        digitalWrite(crankPin, pinValue & 0x01);
        digitalWrite(camPin, (pinValue >> 1) & 0x01);

        if (++currentIndex == Wheels[currentPattern].patternLength)
        {                     // walk the pattern
            currentIndex = 0; // when the end of the pattern is reached go back to the start.
        }
    }

    // If resetPrescaler flag is true, reset pre-scaler based on pre-scalerBits value
    if (resetPrescaler)
    {
        timerAlarmDisable(timer); // Disable the timer alarm
        timerWrite(timer, 0);
        timerSetDivider(timer, prescalerBits);
        timerAttachInterrupt(timer, 0, timerISR);
        timerAlarmEnable(timer);
        resetPrescaler = false; // Reset resetPrescaler flag
    }

    timerAlarmWrite(timer, new_OCR1A, true); // Set OCR1A to new_OCR1A for RPM changes
    portEXIT_CRITICAL_ISR(&timerMux);        // Exit the critical section
}

void updateEncoder()
{
    //  Only missing for debug!!
}

// Holds various instructions needed each time the pattern is changed
void patternCheck()
{
        //  Only missing for debug!!
}

// Define a function to initialize board hardware
void initBoard()
{
    timer = timerBegin(0, 80, true);              // Use Timer 0 with 80 MHz clock, and count up
    timerAttachInterrupt(timer, &timerISR, true); // Attach the ISR function
    timerAlarmWrite(timer, 1, true);              // Set the timer alarm value to 1
    timerAlarmEnable(timer);                      // Enable the timer alarm
    reset_new_OCR1A(desiredRPM);                  // Reset the timer pre-scaler

    // Configure pin modes for inputs and outputs
    // Set pins 8,9,10 as outputs
    pinMode(crankPin, OUTPUT); // set crankPin as an output
    pinMode(camPin, OUTPUT);   // set camPin as an output

    pinMode(rpmPot, INPUT); // set potentiometer as an input
    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE); // Interrupts for rotary encoder
    attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE); // Interrupts for rotary encoder

    // COMMENTED OUT FOR DEBUG
    // EEPROM.get(0, currentPattern); // Get previously stored pattern

    // Check if the loaded value is within the range.
    if (currentPattern < minWheels || currentPattern > MAX_WHEELS)
    {
        currentPattern = 11; // If not use a default value of 11
    }
}

// This function gets bitshift value based on pre-scaler enum
uint8_t get_bitshift_from_prescaler(uint8_t *prescalerBits)
{
    switch (*prescalerBits)
    { // Check prescaler enum and assign corresponding bitshift value
    case PRESCALE_1024:
        return 10;
    case PRESCALE_256:
        return 8;
    case PRESCALE_64:
        return 6;
    case PRESCALE_8:
        return 3;
    case PRESCALE_1:
        return 0;
    }
    return 0; // Return 0 if no corresponding bitshift value is found
}

// This function takes a pointer to a potential OCR1A value, and two pointers to variables representing the prescaler and bitshift values
// The function then determines the correct prescaler and bitshift values based on the given OCR1A value
void get_prescaler_bits(uint32_t *potential_oc_value, uint8_t *prescaler, uint8_t *bitshift)
{
    // Assign corresponding prescaler enum and bitshift values based on OCR1A value
    if (*potential_oc_value >= 16777216)
    {
        // If OCR1A value is greater than or equal to 16777216
        *prescaler = PRESCALE_1024;
        *bitshift = 10;
    }
    else if (*potential_oc_value >= 4194304)
    {
        // If OCR1A value is greater than or equal to 4194304
        *prescaler = PRESCALE_256;
        *bitshift = 8;
    }
    else if (*potential_oc_value >= 524288)
    {
        // If OCR1A value is greater than or equal to 524288
        *prescaler = PRESCALE_64;
        *bitshift = 6;
    }
    else if (*potential_oc_value >= 65536)
    {
        // If OCR1A value is greater than or equal to 65536
        *prescaler = PRESCALE_8;
        *bitshift = 3;
    }
    else
    {
        // If OCR1A value is less than 65536
        *prescaler = PRESCALE_1;
        *bitshift = 0;
    }
}

// This function sets new OCR1A value based on target RPM
void reset_new_OCR1A(uint32_t new_rpm)
{
    // Store temporary variables
    uint32_t tmp;
    uint8_t bitshift;
    uint8_t tmp_prescaler_bits;

    // Calculate new OCR1A value based on current pattern and target RPM
    tmp = (uint32_t)(8000000.0 / (Wheels[currentPattern].rpm_scaler * (float)(new_rpm < 10 ? 10 : new_rpm)));

    get_prescaler_bits(&tmp, &tmp_prescaler_bits, &bitshift); // Determine prescaler and bitshift values based on OCR1A value
    new_OCR1A = (uint16_t)(tmp >> bitshift);                  // Update new OCR1A value based on prescaler and bitshift values
    prescalerBits = tmp_prescaler_bits;                       // Update prescalerBits with value stored in tmp_prescaler_bits
    resetPrescaler = true;                                    // Set resetPrescaler flag to true
}

void adc()
{
    // Check if adc0_Ready flag is true
    adc0 = analogRead(rpmPot);
    // Left shift the value of adc0 by tmpRPM_Shift bits and assign it to tempRPM
    tempRPM = adc0 << tmpRPM_Shift;
    // Check if tempRPM is greater than maxRPM
    // If it is, assign the value of maxRPM to tempRPM, else keep it as is
    tempRPM = (tempRPM > maxRPM) ? maxRPM : tempRPM;
    desiredRPM = tempRPM;     // Assign the value of tempRPM to the desiredRPM variable
    reset_new_OCR1A(tempRPM); // Call the reset_new_OCR1A function with tempRPM as argument
}
#endif