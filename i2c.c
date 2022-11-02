/*
 * i2c.c
 *
*/

#include "i2c.h"

// current config of the selectIO (cause we don't want to overwrite the top LED bit)
uint8_t selectIOCurrent = 0x0;

void i2cInit(void) {
    cmr_i2cInit(&bmb_i2c, I2C1,
                400000, 0, // 100kHz limited by the PCA9536 TODO: Check if own address should be 0
                GPIOB, GPIO_PIN_8, // clock
                GPIOB, GPIO_PIN_9); // data

    for (int bmb = 0; bmb < NUM_BMBS; bmb++) {
        for (int side = 0; side < 2; side++) {
            i2c_enableI2CMux(bmb, side);
            i2c_configSelectMux();
            i2c_configADC();
        }
        i2c_disableI2CMux(bmb);
    }
}

// This verifies everything looks like it works
// and configures everything
// A non-zero return value indicates error, higher 8 bits
// indicates the mux that produced the error, then lower 8
// bits 0 indicates mux failure, 1 indicates 4 IO expander
// failure, 2 indicates ADC failure, 3 indicates 9 IO expander
// failure
uint16_t i2cVerifyConfigChain(void) {
    for (int bmb = 0; bmb < NUM_BMBS; bmb++) {
        uint8_t data = 0;
        if (cmr_i2cRX(&bmb_i2c, bms_mux_address[bmb], &data,
                  1, I2C_TIMEOUT) != 0) {
            return ((uint16_t)(i)) << 8;
        }
        if (data != 0x0) {
            return ((uint16_t)(i)) << 8;
        }

        for (int side = 0; side < 2; side++) {
            // verified mux is reset to 0
            // now enable the mux, check ADC and GPIO per side
            i2c_enableI2CMux(bmb, side);
            // now verify the 4 IO expander
            //if (
        }
    }
    return 0;
}

bool i2c_enableI2CMux(uint8_t bmb, uint8_t side) {
    configASSERT(side <= 1);
    configASSERT(bmb < NUM_BMBS);
    // bit 2 is enable bit, bit 1 & 0 is the side (either 00 or 01)
    uint8_t data = 0x4 | side;
    if(cmr_i2cTX(&bmb_i2c, bms_mux_address[bmb], &data, 1, I2C_TIMEOUT) != 0) {
        return false;
    }
    return true;
}

bool i2c_disableI2CMux(uint8_t bmb) {
    configASSERT(bmb < NUM_BMBS);
    // bit 2 is enable bit
    uint8_t data = 0x0;
    if(cmr_i2cTX(&bmb_i2c, bms_mux_address[bmb], &data, 1, I2C_TIMEOUT) != 0) {
        return false;
    }
    return true;
}

bool i2c_configSelectMux() {
    // select control register, set them all to output
    uint8_t data[2] = {0x3, 0x00};
    if (cmr_i2cTX(&bmb_i2c, BMS_SELECT_IO_ADDR, &data, 2, I2C_TIMEOUT) != 0) {
        return false;
    }
    return true;
}

bool i2c_select4MuxChannel(uint8_t channel) {
    configASSERT(channel < 4);
    // 0x1 is output port, we set select lines of mux
    // mux only uses last 2 bits, the top 4th bit is the LED blinking
    // save top 2 bits, overwrite bottom 2 bits
    selectIOCurrent = (selectIOCurrent & 0xC) | channel;
    uint8_t outData[2] = {0x1, selectIOCurrent};
    if(cmr_i2cTX(&bmb_i2c, BMS_SELECT_IO_ADDR, &outData, 2, I2C_TIMEOUT) != 0) {
        return false;
    }
    return true;
}

bool i2c_selectMuxBlink() {
    // flip top 2 bits, don't flip bottom 2 bits
    selectIOCurrent = (~selectIOCurrent & 0xC) | (selectIOCurrent & 0x3);
    uint8_t outData[2] = {0x1, selectIOCurrent};
    if(cmr_i2cTX(&bmb_i2c, BMS_SELECT_IO_ADDR, &outData, 2, I2C_TIMEOUT) != 0) {
        return false;
    }
    return true;
}

bool i2c_configADC() {
    // 1111 means {setup_bit, internal_ref, ref_output, ref_always_on}
    // 0010 means {internal_clock, unipolar, no_action, X}
    uint8_t setupByte = 0xF2;
    // 0_00_0111_1 means {config_bit, scan_all, scan_to_A7, single_ended}
    uint8_t configByte = 0x1F;
    if(cmr_i2cTX(&bmb_i2c, BMS_ADC_ADDR, &setupByte, 1, I2C_TIMEOUT) != 0) {
        return false;
    if(cmr_i2cTX(&bmb_i2c, BMS_ADC_ADDR, &configByte, 1, I2C_TIMEOUT) != 0) {
        return false;
    }
    return true;
}

bool i2c_scanADC(int16_t adcResponse[]) {
    uint8_t buffer[16];
    if(cmr_i2cRX(&bmb_i2c, BMS_ADC_ADDR, buffer, 16, I2C_TIMEOUT) != 0) {
        return false;
    }   
    for(int i = 0; i < 8; i++) {
        // top 6 bits should be 1
        if (~(buffer[i<<1] >> 2) != 0x0) {
            return false;
        }
        adcResponse[i] = ((((uint16_t)buffer[i<<1]) << 8) | (buffer[(i<<1)+1])) & 0x3FF;
    }
    return true;
}
