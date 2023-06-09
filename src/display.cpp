#include <Arduino.h>
#include "display.h"
#include "structures.h"

// Doesn't work properly. Need to learn to use esp32 timers
#if defined(ESP32)
#include "board_esp32.h"
#endif

// Works well
#if defined(AVR328)
#include "board_avr328.h"
#endif

// Works well
#if defined(PICO)
#include "board_pico.h"
#endif

#if defined(USE_TINY4K)
#include <Tiny4kOLED.h>
#include <fonts.h>
// Ardustim Variables
extern wheels Wheels[];
extern uint8_t currentPattern; // Store Currently selected pattern. Stored in EEPROM.

void loadDisplay()
{
    //  Setup OLED Display
    oled.begin(128, 64, sizeof(tiny4koled_init_128x64br), tiny4koled_init_128x64br);
    oled.clear(); // Clear Screen
    oled.on();    // Turn on OLED Screen
    // Shameless Logo Plug
    // oled.setCursor(14, 3);       // Logo Location
    // oled.print("DetonationEMS"); // Logo
    // delay(2500);                 // Let Logo show for 2 seconds
    // oled.clear();                // Clear Screen
}

void updateDisplay()
{
    oled.clear(); // Clear Screen
    oled.setFont(FONT8X16);
    oled.setCursor(0, 0);
    oled.print("Pattern: ");
    oled.print(currentPattern + 1); // Add one to exclude 0 from pattern count
    oled.print("/");
    oled.print(MAX_WHEELS);
    oled.setCursor(0, 10);
    char c;
    uint8_t row = 1; // Initialize row number
    for (uint8_t j = 0; (c = pgm_read_byte(&(Wheels[currentPattern].patternName[j]))) != '\0'; j++)
    {
        if (j > 15 * row)
        {          // Check if we need to move to the next row
            row++; // Increment row number
            oled.setCursor(0, 10 * row);
        }
        oled.print(c);
    }
}
#endif

#if defined(ADAFRUIT_GFX)
#include <hardware/spi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_WIDTH 128
#define OLED_HEIGHT 64

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
// Ardustim Variables
extern wheels Wheels[];
extern uint8_t currentPattern; // Store Currently selected pattern. Stored in EEPROM.

void loadDisplay()
{
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Pattern: ");
    display.print(currentPattern + 1); // Add one to exclude 0 from pattern count
    display.print("/");
    display.print(MAX_WHEELS);
    display.setCursor(0, 10);
    display.display();
}
void updateDisplay()
{
    char buf[80];

    display.clearDisplay();              // Clear the display buffer
    display.setTextSize(1);              // Set text size to 1 (6x8 pixels per character)
    display.setTextColor(SSD1306_WHITE); // Set text color to white
    display.setCursor(0, 0);             // Set cursor to top-left corner

    // Wheel names are then sent 1 per line
    for (byte x = 0; x < MAX_WHEELS; x++)
    {
        strcpy_P(buf, Wheels[x].patternName);
        display.println(buf);
    }
    // display.println(Wheels[currentPattern].patternName); // Print shameless plug to display
    display.display(); // Update OLED display
}

#endif