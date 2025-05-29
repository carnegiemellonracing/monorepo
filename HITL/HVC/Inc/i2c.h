/**
  * @file i2c.h
  * @brief Header for I2C interface to HITL's ADC and DAC chips
  * @author Carnegie Mellon Racing
**/

#ifndef I2C_H
#define I2C_H

#include <CMR/i2c.h>
#include <stdbool.h>
#include <stdint.h>

// #define I2C_NUM_BMBS 8
// I2c addresses of muxes 
// From https://www.nxp.com/docs/en/data-sheet/PCA9542A.pdf
// #define BMS_MUX_BASE_ADDR 0x70

// https://www.mouser.com/datasheet/2/302/PCA9536-1127758.pdf
// #define BMS_SELECT_IO_ADDR 0x41
// https://www.mouser.com/datasheet/2/256/MAX11606-MAX11611-1514405.pdf
// #define BMS_ADC_ADDR 0x33
// #define BMS_CELL_BALANCE_IO_ADDR 0x50 // TODO: Change this, variable from 0x50-0x57

// This needs to be reasonably long b/c the ADC takes a while to send back
// With a too low threshold, you will panic on the I2C HAL state not being ready
#define I2C_TIMEOUT 10

#define ADC1_ADDR 0b1001000 // modify last 2 bits according to address bit inputs on PCB
#define ADC2_ADDR 0b1001001 // modify last 2 bits according to address bit inputs on PCB
#define DAC1_ADDR 0b0010000 // modify according to address bit inputs on PCB
#define DAC2_ADDR 0b0010000 // modify according to address bit inputs on PCB


bool i2cInit();
void resetClock();

uint16_t i2c_readI2CADC1(uint8_t channel);
uint16_t i2c_readI2CADC2(uint8_t channel);

bool i2c_writeExtRefDAC1();

bool i2c_writeI2CDAC1(uint8_t channel, uint8_t value);
bool i2c_writeI2CDAC2(uint8_t channel, uint8_t value);


#endif