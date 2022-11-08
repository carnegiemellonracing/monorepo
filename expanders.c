/**
 * @file expanders.c
 * @brief GPIO expanders implementation
 *
 * @author Carnegie Mellon Racing
 */

#include <CMR/i2c.h>    // I2C interface

#include "expanders.h"   // Interface to implement

/** @brief Primary I2C interface */
static cmr_i2c_t i2c;

/** @brief DIM I2C address */
static uint32_t ownAddress = 0x00; // 0x00 = 0b0000000

/** @brief GPIO expander I2C address */
static uint16_t gpioExpanderAddress = 0x27; // 0x27 = 0b0100111

/** @brief GPIO expander output port 1 command byte */
static uint8_t outputCommand = 0x03; // 0x03 = 0b00000011

/**
 * @brief Initializes the GPIO expander interface.
 */
void expandersInit(void) {
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

}



