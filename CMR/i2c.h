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

extern const I2C_CLOCK_LOW;
extern const I2C_CLOCK_HI;

typedef struct {
	I2C_HandleTypeDef handle;      /**< @brief HAL I2C Handle.*/
}cmr_i2c_t;

int cmr_i2cTX(uint16_t devAddr, uint8_t *data, int dataLength);

int cmr_i2cRX(uint16_t devAddr, uint8_t *data, int dataLength);

void cmr_i2cInit(
    cmr_i2c_t *i2c, I2C_Typedef *instance,
    uint32_t clockSpeed, uint32_t ownAddr,
    GPIO_Typedef *i2cClkPort, uint32_t i2cClkPin,
    GPIO_Typedef *i2cDataPort, uint32_t i2cDataPin
);

#endif /* CMR_I2C_H_ */
