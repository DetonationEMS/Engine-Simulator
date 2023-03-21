#if defined(AVR328)
#include <avr/pgmspace.h>
#include <stdint.h>
#include <EEPROM.h>

#include "board_avr328.h"
#include "wheel_defs.h"
#include "structures.h"
#include "display.h"
#include "wheel_gen.h"

// Rotary Encoder pins
#define encoderPinA 2
#define encoderPinB 3
// #define encoderSwitch 4

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
uint8_t currentPattern;    // Store Currently selected pattern. Stored in EEPROM.
uint16_t currentIndex = 0; // Store currentPattern's indexed value. This value must be set to zero each time the pattern changes, starts or stops.
extern uint16_t patternLength;
// Flag for updating the display
bool updateDisplayName = true;

// This function will be used to generate trigger arrays when selected. (Very incomplete) saving large amounts of space
void generate_array(unsigned char *arr)
{

  if (currentPattern == 12)
  {
    generate_fourty_minus_one(arr);
  }
  if (currentPattern == 13)
  {
    generate_dizzy_four_trigger_return(arr);
  }
  if (currentPattern == 14)
  {
    generate_oddfire_vr(arr);
  }
  if (currentPattern == 15)
  {
    generate_optispark_lt1(arr);
  }
}

// This function updates the rotary encoder's current position and current pattern
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
  // Generate the array based on a selection
  unsigned char generated_array[Wheels[currentPattern].patternLength]; // Set maximum array size. (720x2=1440) I cannot imagine a larger array.
  generate_array(generated_array);

  lastEncoded = encoded;         // Update lastEncoded to match encoded
  EEPROM.put(0, currentPattern); // Store currentPattern to EEPROM

  ISR_loop = true; // Restart the loop
}

// Holds various instructions needed each time the pattern is changed
void patternCheck()
{
  // Checks flag to see if display needs to be updated (has to be better way to do this)
  if (updateDisplayName == true) // Called if new pattern needs to be displayed.
  {
    updateDisplayName = false;
    updateDisplay(); // Calls function that updates display to new pattern
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
  cli(); // Disable global interrupts
  // Configure timer hardware for RPM control
  TCCR1A = 0;              // Set Timer/Counter 1 Control Register A to 0 (Normal port operation, CTC mode disabled)
  TCCR1B = 0;              // Set Timer/Counter 1 Control Register B to 0 (Normal port operation, CTC mode disabled)
  TCNT1 = 0;               // Reset Timer/Counter 1 value to 0
  OCR1A = 1;               // Set Timer/Counter 1 Output Compare Register A to 1
  TCCR1B |= (1 << WGM12);  // Set the Timer1 mode to CTC (clear timer on compare match)
  TCCR1B |= (1 << CS10);   // Set the Timer1 pre-scaler to 1 (no pre-scaling)
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 overflow interrupt

  // Configure analog input for RPM control
  ADMUX &= B11011111;  // Set the reference voltage to AVCC (5V) for the ADC (clearing bit 5)
  ADMUX |= B01000000;  // Set the reference voltage to AVCC (5V) for the ADC (setting bits 6 and 7)
  ADMUX &= B11110000;  // Clear MUX3..0 in ADMUX in preparation for setting analog input
  ADCSRA |= B10000000; // Set ADEN in ADCSRA to enable the ADC.
  ADCSRA |= B00100000; // Set ADATE in ADCSRA to enable auto-triggering.
  ADCSRB &= B11111000; // Clear ADTS2..0 in ADCSRB to set trigger mode to free running.
  ADCSRA |= B00000111; // Set the ADC pre-scaler to 128 (sampling frequency of 125 kHz)
  ADCSRA |= B00001000; // Set ADIE in ADCSRA to enable the ADC interrupt.

  sei(); // Enable global interrupts

  ADCSRA |= B01000000; // Start the ADC conversion

  reset_new_OCR1A(desiredRPM); // Reset the timer pre-scaler

  // Configure pin modes for inputs and outputs
  // Set pins 8,9,10 as outputs
  DDRB |= (1 << DDB0); // Set PB0 (pin 8) as output
  DDRB |= (1 << DDB1); // Set PB1 (pin 9) as output
  DDRB |= (1 << DDB2); // Set PB2 (pin 10) as output (Knock signal on LS1 pattern, not implemented. Needs to be enabled to prevent errors when using "gm_ls1_crank_and_cam")

  // Set pins 2 and 3 as inputs with pull-ups and interrupts.
  DDRD &= ~(1 << DDD3); // Set D3 (pin 3) as input
  PORTD |= (1 << PD3);  // Enable pull-up on D3 (pin 3)
  DDRD &= ~(1 << DDD2); // Set D2 (pin 2) as input
  PORTD |= (1 << PD2);  // Enable pull-up on D2 (pin 2)

  // pinMode(encoderSwitch, INPUT_PULLUP);                                      // Rotary encoder switch
  // attachInterrupt(digitalPinToInterrupt(encoderSwitch), encoderButton, LOW); // Interrupts for rotary encoder push button (not sure how well this will work)

  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE); // Interrupts for rotary encoder
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE); // Interrupts for rotary encoder

  EEPROM.get(0, currentPattern); // Get previously stored pattern

  // Check if the loaded value is within the range.
  if (currentPattern < minWheels || currentPattern > MAX_WHEELS)
  {
    currentPattern = 11; // If not use a default value of 11
  }
}

void adc()
{
  // Check if adc0_Ready flag is true
  if (adc0_Ready)
  {
    adc0_Ready = false; // Reset adc0_Ready flag to false
    // Left shift the value of adc0 by tmpRPM_Shift bits and assign it to tempRPM
    tempRPM = adc0 <<= tmpRPM_Shift;
    // Check if tempRPM is greater than maxRPM
    // If it is, assign the value of maxRPM to tempRPM, else keep it as is
    tempRPM = (tempRPM > maxRPM) ? maxRPM : tempRPM;
    desiredRPM = tempRPM;     // Assign the value of tempRPM to the desiredRPM variable
    reset_new_OCR1A(tempRPM); // Call the reset_new_OCR1A function with tempRPM as argument
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

// This is an interrupt service routine triggered by TIMER1_COMPA_vect interrupt
ISR(TIMER1_COMPA_vect)
{
  TCNT1 = 0; // Reset the timer

  // If triggerOutput flag is true, stream indexed data and control output speed
  if (triggerOutput == true)
  {
    PORTB = pgm_read_byte(&Wheels[currentPattern].selectedPattern[currentIndex]); // Stream indexed data

    if (++currentIndex == Wheels[currentPattern].patternLength) // Walk the pattern
    {
      currentIndex = 0; // At end of array go back to the beginning.
    }
  }

  // If resetPrescaler flag is true, reset prescaler based on prescalerBits value
  if (resetPrescaler)
  {
    TCCR1B &= ~((1 << CS10) | (1 << CS11) | (1 << CS12)); // Clear CS10, CS11 and CS12 bits
    TCCR1B |= prescalerBits;                              // Set prescaler based on prescalerBits value
    resetPrescaler = false;                               // Reset resetPrescaler flag
  }
  OCR1A = new_OCR1A; // Set OCR1A to new_OCR1A for RPM changes
}

// This is an Interrupt Service Routine (ISR) for Analog to Digital Converter (ADC) Interrupt vector
ISR(ADC_vect)
{
  if (analogPort == 0) // Check if the current ADC port is 0
  {
    adc0 = ADCL | (ADCH << 8); // Read the ADC value for port 0
    adc0_Ready = true;         // Set the flag to indicate the ADC value for port 0 is ready
    return;
  }
}
#endif