/**
 * @file ledStrip.c
 * @brief LED strip implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <CMR/i2c.h>    // I2C interface

#include "ledStrip.h"   // Interface to implement

static const uint8_t ledOutputRegisters[] = {
    0x00, // 0x00 = 0b00000000, no LEDs on
    0x02, // 0x02 = 0b00000010, LED 1 on
    0x03, // 0x03 = 0b00000011, LEDs 1-2 on
    0x80, // 0x80 = 0b10000000, LED 3 on
    0xC0, // 0xC0 = 0b11000000, LEDs 3-4 on
    0xE0, // 0xE0 = 0b11100000, LEDs 3-5 on
    0xF0, // 0xF0 = 0b11110000, LEDs 3-6 on
    0xF8, // 0xF8 = 0b11111000, LEDs 3-7 on
    0xFC, // 0xFC = 0b11111100, LEDs 3-8 on
    0xFE, // 0xFE = 0b11111110, LEDs 3-9 on
    0xFF  // 0xFF = 0b11111111, LEDs 3-10 on
};

/** @brief Primary I2C interface */
static cmr_i2c_t i2c;

/** @brief DIM I2C address */
static uint32_t ownAddress = 0x50; // 0x50 = 0b1010000

/** @brief GPIO expander I2C address */
static uint16_t gpioExpanderAddress = 0x20; // 0x20 = 0b0100000

/** @brief GPIO expander output port 1 command byte */
static uint8_t outputCommand = 0x03; // 0x03 = 0b00000011

/**
 * @brief Initializes the LED strip interface.
 */
void ledStripInit(void) {
    // I2C initialization
    cmr_i2cInit(
        &i2c, I2C1,
        I2C_CLOCK_LOW, ownAddress,
        GPIOB, GPIO_PIN_6,
        GPIOB, GPIO_PIN_7
    );
    // Set ports to output
    uint8_t data[3] = {
        0x06,
        0x00,
        0x00
    };
    cmr_i2cTX(&i2c, gpioExpanderAddress, data, 3, 1);
    setNumLeds(0);
}

/**
 * @brief Turns on the specified number of LEDs.
 *
 * @param numLeds the number of LEDs to turn on
 */
void setNumLeds(unsigned int numLeds) {
    // Check within range
    if (numLeds > NUM_LEDS) {
        numLeds = NUM_LEDS;
    }

    // Array for I2C transmission data
    // first byte is the command byte
    // second byte is sent to output port 1
    // third byte is sent to output port 0
    uint8_t data[3];

    // Set command byte
    data[0] = outputCommand;
    
    // Set LED output registers
    // LEDs 1-2 are on port 1, while 3-10 are on port 0,
    // so the data must be sent separately
    if (numLeds > 2) {
        data[1] = ledOutputRegisters[2];
        data[2] = ledOutputRegisters[numLeds];
    } else {
        data[1] = ledOutputRegisters[numLeds];
        data[2] = ledOutputRegisters[0];
    }

    // Tranmit data to the GPIO expander
    cmr_i2cTX(&i2c, gpioExpanderAddress, data, 3, 1);
}


