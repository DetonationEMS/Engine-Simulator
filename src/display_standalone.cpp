#if defined(STANDALONE_DISPLAY)
#include <Arduino.h>
#include <avr/pgmspace.h>

// #include <pico/stdlib.h>
#include "pico.h"
#include <cstdint>
#include <stdio.h>
#include <hardware/adc.h>
#include <hardware/gpio.h>



#define I2C_PORT i2c0
#define I2C_ADDR 0x3C

const uint8_t SCREEN_WIDTH = 128;
const uint8_t SCREEN_HEIGHT = 64;
const uint8_t OLED_DATA_MODE = 0x40;
const uint8_t OLED_CMD_MODE = 0x00;

uint8_t buffer[SCREEN_WIDTH * SCREEN_HEIGHT / 8];

void sendCommand(uint8_t command) {
    uint8_t tx_data[2] = { OLED_CMD_MODE, command };
    i2c_write_blocking(I2C_PORT, I2C_ADDR, tx_data, 2, false);
}

void sendData(const uint8_t* data, uint16_t size) {
    uint8_t tx_data[17] = { OLED_DATA_MODE };
    for (int i = 0; i < size; i++) {
        tx_data[i+1] = data[i];
        if ((i+1) % 16 == 0 || i == size-1) {
            i2c_write_blocking(I2C_PORT, I2C_ADDR, tx_data, i+2, false);
            tx_data[0] = OLED_DATA_MODE;
        }
    }
}

void initOled() {
    sendCommand(0xAE); // Display off
    sendCommand(0xD5); // Set display clock divide ratio/oscillator frequency
    sendCommand(0x80);
    sendCommand(0xA8); // Set multiplex ratio
    sendCommand(SCREEN_HEIGHT - 1);
    sendCommand(0xD3); // Set display offset
    sendCommand(0x00);
    sendCommand(0x40); // Set start line
    sendCommand(0x8D); // Charge pump
    sendCommand(0x14);
    sendCommand(0x20); // Set memory mode
    sendCommand(0x00);
    sendCommand(0xA1); // Set segment re-map
    sendCommand(0xC8); // Set COM output scan direction
    sendCommand(0xDA); // Set COM pins hardware configuration
    sendCommand(0x12);
    sendCommand(0x81); // Set contrast
    sendCommand(0xCF);
    sendCommand(0xD9); // Set precharge period
    sendCommand(0xF1);
    sendCommand(0xDB); // Set VCOMH deselect level
    sendCommand(0x40);
    sendCommand(0xA4); // Entire display on
    sendCommand(0xA6); // Set normal display
    sendCommand(0xAF); // Display on
}

void clearDisplay() {
    memset(buffer, 0, sizeof(buffer));
    sendData(buffer, sizeof(buffer));
}

void setPixel(uint8_t x, uint8_t y, bool value) {
    if (x >= SCREEN_WIDTH || y >= SCREEN_HEIGHT) {
        return;
    }

    if (value) {
        buffer[x + (y / 8) * SCREEN_WIDTH] |= (1 << (y % 8));
    } else {
        buffer[x + (y / 8) * SCREEN_WIDTH] &= ~(1 << (y % 8));
    }
}

int main() {

    _i2c_init(I2C_PORT, 400000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    i2c_set_slave_mode(I2C_PORT, false, I2C_ADDR);
    // i2c_set_timeout_us(I2C_PORT, 500);

    initOled();
    clearDisplay();

    while (true) {
        // Set some pixels on the display
        setPixel(10, 10, true);
        setPixel(20, 20, true);
        setPixel(30, 30, true);
        setPixel(40, 40, true);
        setPixel(50, 50, true);

        // Send the pixel data to the display
        sendData(buffer, sizeof(buffer));

        sleep_ms(1000);

        // Clear the display
        clearDisplay();

        // Wait for a second before setting pixels again
        sleep_ms(1000);
    }

    return 0;
}

#endif