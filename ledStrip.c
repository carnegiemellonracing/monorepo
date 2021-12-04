/**
 * @file ledStrip.c
 * @brief LED strip implementation.
 *
 * @author Carnegie Mellon Racing
 */

#include <CMR/i2c.h>    // I2C interface

#include "ledStrip.h"   // Interface to implement


/** @brief Primary I2C interface */
static cmr_i2c_t i2c;

/** @brief DIM I2C address */
static uint32_t ownAddress = 0x50; // 0x50 = 0b1010000

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
}

/**
 * @brief Reads an ADC channel's value.
 *
 * @param channel The channel.
 * @return The channel's last sampled value.
 */
void setNumLeds(unsigned int numLeds) {
    
}


