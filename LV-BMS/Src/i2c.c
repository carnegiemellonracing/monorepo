/*
 * i2c.c
 *
 *  Created on: Sep 18, 2023
 *      Author: CMR
 */

#include "i2c.h"
#include <stdbool.h>
#include <stm32f4xx.h>
#define HAL_I2C_MODULE_ENABLED


void i2c_init() {
	cmr_i2cInit(&i2c, I2C1, I2C_CLOCK_LOW, HOME_ADDR, GPIOB, GPIO_PIN_6, GPIOB, GPIO_PIN_7);
}
// functions return 0 = success, 1 = error.
uint8_t i2c_read_register(uint8_t reg_address, uint8_t *value) {
    if (cmr_i2cRX(&i2c, reg_address, value, 1, I2C_TIMEOUT)) {
        return 1;
    }
    return 0;
}

uint8_t i2c_write_register(uint8_t reg_address, uint8_t value) {
    if (cmr_i2cTX(&i2c, reg_address, &value, 1, I2C_TIMEOUT)) {
        return 1;
    }
    return 0;
}

// Func to configure the amplifier gain in CONFIG_1
uint8_t set_gain(uint8_t gain) {
    uint8_t config_value;
    if (i2c_read_register(CONFIG_1, &config_value)) {
        return 1;  // Return an error if the read operation fails
    }
    if (gain == 4) {
        config_value &= ~0x01;
    } else if (gain == 8) {
        config_value |= 0x01;
    }
    return i2c_write_register(CONFIG_1, config_value);
}

// Function to switch measurement mode (SENSEP or SENSEN)
uint8_t set_measurement_mode(bool measure_sensep) {
    uint8_t config_value;
    if (i2c_read_register(CONFIG_1, &config_value)) {
        return 1;
    }
    if (measure_sensep) {
        config_value |= 0x04;  // Set D2 to 1 (SENSEP mode)
    } else {
        config_value &= ~0x04;  // Clear D2 to 0 (SENSEN mode)
    }
    return i2c_write_register(CONFIG_1, config_value);
}

uint8_t write_sleep() {
	uint8_t msg = 0x80;
	return cmr_i2cTX(&i2c, 0x25, &msg, 1, I2C_TIMEOUT);
}

// Function to validate a register write
uint8_t i2c_write_and_validate(uint8_t reg_address, uint8_t value) {
    if (i2c_write_register(reg_address, value)) {
        return 1;
    }
    uint8_t read_value;
    if (i2c_read_register(reg_address, &read_value)) {
        return 1;  // Read failed
    }

    // Check if the written value matches the read value
    return (read_value == value) ? 0 : 1;  // Success if values match
}

/*previous shit
uint8_t read_chip_id() {
	uint8_t chip_id;
	if(cmr_i2cRX(&i2c, 0x27, &chip_id, 1, I2C_TIMEOUT))
		return 0;
	return chip_id;
}

uint8_t read_status() {
	uint8_t status;
	if(cmr_i2cRX(&i2c, 0x20, &status, 1, I2C_TIMEOUT))
		return 0;
	return status;
}

uint8_t read_ref_sel() {
	uint8_t rel_sel;
	if(cmr_i2cRX(&i2c, 0x24, &rel_sel, 1, I2C_TIMEOUT))
		return 0;
	return rel_sel;
}

uint8_t write_ref_sel(bool one_point_five_volts) {
	uint8_t msg = one_point_five_volts ? 0 : 1;
	return cmr_i2cTX(&i2c, 0x24, &msg, 1, I2C_TIMEOUT);
}

uint8_t read_cell_ctl() {
	uint8_t cell_ctl;
	if(cmr_i2cRX(&i2c, 0x21, &cell_ctl, 1, I2C_TIMEOUT))
		return 0;
	return cell_ctl;
}

uint8_t write_cell_ctl(uint8_t VCOUT_SEL, uint8_t CELL_SEL) {
	uint8_t msg = (VCOUT_SEL << 4) | CELL_SEL;
	return cmr_i2cTX(&i2c, 0x21, &msg, 1, I2C_TIMEOUT);
}

uint8_t read_power_ctl() {
	uint8_t power_ctl;
	if(cmr_i2cRX(&i2c, 0x25, &power_ctl, 1, I2C_TIMEOUT))
		return 0;
	return power_ctl;
}
*/


