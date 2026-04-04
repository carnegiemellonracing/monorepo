/*
 * i2c.c
 *
 *  Created on: Jan 24, 2026
 *      Author: CMR
 */

#include "i2c.h"
#include <stdbool.h>
#include <stm32f4xx.h>
#define HAL_I2C_MODULE_ENABLED


void i2c_slave_init() {
	cmr_i2cInit(&i2c, I2C1, I2C_CLOCK_LOW, HOME_ADDR, GPIOB, GPIO_PIN_6, GPIOB, GPIO_PIN_7);
    cmr_i2cSlaveInit(&i2c, I2C1);
}
// functions return 0 = success, 1 = error.
uint8_t i2c_slave_rx(cmr_i2c_t *i2c, uint8_t *rxData, size_t len) {
    if (cmr_i2cSlaveRx(i2c, rxData, len)) {
        return 1;
    }
    return 0;
}

uint8_t i2c_slave_tx(cmr_i2c_t *i2c, uint8_t *txData, size_t len) {
    if (cmr_i2cSlaveTx(i2c, txData, len)) {
        return 1;
    }
    return 0;
}

uint8_t write_66_test(cmr_i2c_t *i2c) {
    static uint8_t txData[1] = {0x66};
    return i2c_slave_tx(i2c, txData, sizeof(txData));
}

uint8_t read_8bytes_test(cmr_i2c_t *i2c) {
    static uint8_t rxData[8];
    i2c_slave_rx(i2c, rxData, sizeof(rxData));
    // add a delay??
    // print_bytes(i2c->slaveRxBuff, i2c->slaveRxLen);
    return 0;
}


// uint8_t write_sleep() {
// 	uint8_t msg = 0x80;
// 	return cmr_i2cTX(&i2c, 0x25, &msg, 1, I2C_TIMEOUT);
// }