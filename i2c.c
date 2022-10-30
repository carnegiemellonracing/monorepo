/*
 * i2c.c
 *
*/

#include "i2c.h"

void i2cInit(void) {
    cmr_i2cInit(&bmb_i2c, I2C1,
                100000, 0, // 100kHz limited by the PCA9536 TODO: Check if own address should be 0
                GPIOB, GPIO_PIN_8, // clock
                GPIOB, GPIO_PIN_9); // data
}

// This verifies everything looks like it works
// and configures everything
// A non-zero return value indicates error, higher 8 bits
// indicates the mux that produced the error, then lower 8
// bits 0 indicates mux failure, 1 indicates 4 IO expander
// failure, 2 indicates ADC failure, 3 indicates 9 IO expander
// failure
uint16_t i2cVerifyConfigChain(void) {
    for (uint8_t i = 0; i < NUM_BMBS; i++) {
        uint8_t data = 0;
        if (cmr_i2cRX(&bmb_i2c, bms_mux_address[i], &data,
                  1, I2C_TIMEOUT) != 0) {
            return ((uint16_t)(i)) << 8;
        }
        if (data != 0x0) {
            return ((uint16_t)(i)) << 8;
        }
        // verified mux is reset to 0
        // now enable the mux, select the 4 IO expander
        data = 0x04; // selects channel 0 (0x05 is channel 1)
        if (cmr_i2cTX(&bmb_i2c, bms_mux_address[i], &data,
                  1, I2C_TIMEOUT) != 0) {
            return ((uint16_t)(i)) << 8;
        }
        // now verify the 4 IO expander
        //if (
    }
    return 0;
}


