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
#include <stdbool.h>

#define I2C_NUM_BMBS 8
// #define I2C_NUM_BMBS 1 //TODO: SET THIS BACK TO 8
// I2c addresses of muxes 
// From https://www.nxp.com/docs/en/data-sheet/PCA9542A.pdf
#define BMS_MUX_BASE_ADDR 0x70;
static const uint16_t bms_cell_balancer_addresses[I2C_NUM_BMBS] = {
    0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57
};

// https://www.mouser.com/datasheet/2/302/PCA9536-1127758.pdf
#define BMS_SELECT_IO_ADDR 0x41
// https://www.mouser.com/datasheet/2/256/MAX11606-MAX11611-1514405.pdf
#define BMS_ADC_ADDR 0x33
#define BMS_CELL_BALANCE_IO_ADDR 0x50 // TODO: Change this, variable from 0x50-0x57

#define I2C_TIMEOUT 50

bool i2cInit();

//switch across sides of BMB
bool i2c_enableI2CMux(uint8_t bmb, uint8_t side);
bool i2c_readI2CMux(uint8_t bmb, uint8_t *enabled, uint8_t *side);
bool i2c_disableI2CMux(uint8_t bmb);

//switch mux channel for FET muxes through the IO expander
bool i2c_configSelectMux();
bool i2c_select4MuxChannel(uint8_t channel);
bool i2c_selectMuxBlink();

//get values from each adc channel
bool i2c_configADC();
bool i2c_scanADC(int16_t adcResponse[]);
bool i2c_pullUpCellBalanceIOExpander(uint8_t bmb);
bool i2c_cellBalance(uint8_t bmb, uint8_t cells, uint8_t cells1);



#endif
