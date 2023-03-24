// DOES NOT WORK.

#if defined(PICO)
/* 
Pico Notes(apply to me personally):
May be able to use some tricks from the ESP32 attempt
Pico notes in binder #3
*/

// Define a function to initialize board hardware
void initBoard();
void adc();
void reset_new_OCR1A();
void patternCheck();
void output();

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