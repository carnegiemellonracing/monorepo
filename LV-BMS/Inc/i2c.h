/*
 * i2c.h
 *
 *  Created on: Sep 18, 2023
 *      Author: CMR
 */

#ifndef I2C_C_
#define I2C_C_

#include <CMR/i2c.h>
#include <stdint.h>
#include <stdbool.h>

// Device Constants
#define HOME_ADDR 0x00
#define REG_CHIP_ID 0x07

// Register Addresses (from datasheet)
#define CONFIG_1 0x23     // CONFIG_1 register: Gain and measurement mode
#define CONFIG_2 0x24     // CONFIG_2 register: Reference voltage settings
#define POWER_CTL 0x25    // POWER_CTL register
#define CELL_CTL 0x21     // CELL_CTL register
#define STATUS 0x20
#define VREF_CAL 0x30
#define VC_CAL_EXT_1 0x37
#define VC_CAL_EXT_2 0x38
#define VREF_CAL_EXT 0x3B
// I2C Timeout

#define I2C_TIMEOUT 100

static cmr_i2c_t i2c;

void i2c_init();

uint8_t write_sleep();

//Generic Register Access Functions
uint8_t i2c_read_register(uint8_t reg_address, uint8_t *value);
uint8_t i2c_write_register(uint8_t reg_address, uint8_t value);

// Functions for Gain and Mode Configuration
uint8_t set_gain(uint8_t gain);
uint8_t set_measurement_mode(bool measure_sensep);

// Optional Validation Function
uint8_t i2c_write_and_validate(uint8_t reg_address, uint8_t value);
uint8_t read_cell_ctl();

#endif /* I2C_C_ */
