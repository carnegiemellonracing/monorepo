/*
 * i2c.h
 *
 *  Created on: Feb 22, 2019
 *      Author: jason
 */
#ifndef CMR_I2C_H_
#define CMR_I2C_H_

#include <stm32f4xx_hal.h>
#include <stddef.h>
#include <stdint.h>



typedef struct {
	I2C_HandleTypeDef handle;     /**< @brief HAL I2C Handle.*/
	uint16_t *addresses;           /**< @brief Configured I2C devices.*/
	size_t addressLen;            /**< @brief Number of I2C devices.*/
}cmr_i2c_t;

void cmr_i2cInit(
    cmr_i2c_t *i2c, I2C_TypeDef *instance,
    uint16_t *addresses, const size_t addressLen
);

#endif /* CMR_I2C_H_ */
