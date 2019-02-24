/**
 * @file i2c.h
 * @brief I2C interface.
 *
 * @author Carnegie Mellon Racing
 */

#ifndef CMR_I2C_H_
#define CMR_I2C_H_

#include <stm32f4xx_hal.h>
#include <stddef.h>
#include <stdint.h>

//is this how defining constants works?
static const uint32_t I2C_CLOCK_LOW = 100000
static const uint32_t I2C_CLOCK_HI = 400000


typedef struct {
	I2C_HandleTypeDef handle;      /**< @brief HAL I2C Handle.*/
	uint16_t *devAddr;           /**< @brief Configured I2C devices.*/
	size_t addrLen;             /**< @brief Number of I2C devices.*/
}cmr_i2c_t;

void cmr_i2cInit(
    cmr_i2c_t *i2c, I2C_Typedef *instance,
    uint32_t clockSpeed, uint32_t ownAddr,
    uint16_t *devAddr, const size_t addrLen
    GPIO_Typedef *i2cPort, uint32_t i2cPin
);

#endif /* CMR_I2C_H_ */
