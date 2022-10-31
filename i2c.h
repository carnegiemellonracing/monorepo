/*
 * i2c.h
 * Header for I2C interface to BMB I2C muxes, GPIO expanders, and ADCs
 * Originally written by Gustav Hanse (gustavh)
 * Edited on Sep 29, 2022
 *
*/

#ifndef I2C_H
#define I2C_H

#include <CMR/i2c.h>

#define NUM_BMBS 8
// I2c addresses of muxes 
// From https://www.nxp.com/docs/en/data-sheet/PCA9542A.pdf
static const uint16_t bms_mux_address[NUM_BMBS] = {
    0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77
};

#define BMS_SELECT_IO_ADDR 0x41;
#define BMS_ADC_ADDR 0x33;
#define BMS_CELL_BALANCE_IO_ADDR 0x50; // TODO: Change this, variable from 0x50-0x57

#define I2C_TIMEOUT 50

static cmr_i2c_t bmb_i2c;


//switch across sides of BMB
bool switchI2CMux(uint8_t side, uint8_t bmb);

//switch mux channel for FET muxes through the IO expander
bool selectMuxChannel(uint8_t channel);

//get values from each adc channel
bool scan_adc(int16_t adcResponse[]);



#endif
